// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.ResourceBundle.Control;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.AccelerationConstraint;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TankVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.external.CheesyDriveHelper;
import frc.external.DriveSignal;
import frc.external.roadrunner.TankLocalizer;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase { 
  public enum DriveMode {
    MotionProfile,
    Normal,
    Disable
  }

  private final GyroSubsystem gyro;
  
  private final WPI_TalonFX rightOne = new WPI_TalonFX(DriveConstants.kRightOneCanId, "canivore");
  private final WPI_TalonFX rightTwo = new WPI_TalonFX(DriveConstants.kRightTwoCanId, "canivore");
  private final WPI_TalonFX rightThree = new WPI_TalonFX(DriveConstants.kRightThreeCanId, "canivore");

  private final WPI_TalonFX leftOne = new WPI_TalonFX(DriveConstants.kLeftOneCanId, "canivore");
  private final WPI_TalonFX leftTwo = new WPI_TalonFX(DriveConstants.kLeftTwoCanId, "canivore");
  private final WPI_TalonFX leftThree = new WPI_TalonFX(DriveConstants.kLeftThreeCanId, "canivore");

  private static final double kCountsPerRev = 2048.0;
  private static final double kGearRatio = 9.5;
  private static final double kWheelCirc = Math.PI * 6.02;
  private static final double kEncToInches = ((kCountsPerRev*kGearRatio)/kWheelCirc);

  private final TankLocalizer localizer = new TankLocalizer(this);
  private final BufferedTrajectoryPointStream trajStream = new BufferedTrajectoryPointStream();

  private static final double kMaxVel = encToInches(18000.0 * 10.0) * .9; // 18000/100ms
  private static final double kMaxAngVel = Math.toRadians(90);
  private static final double kTrackWidth = 24.125;
  private static final double kMaxAccel = kMaxVel * 2;

  private static final TrajectoryVelocityConstraint kVelConstraint = getVelocityConstraint(kMaxVel, kMaxAngVel, kTrackWidth);
  private static final TrajectoryAccelerationConstraint kAccelConstraint = getAccelerationConstraint(kMaxAccel);

  private Pose2d currentPose;
  
  public DriveSubsystem(GyroSubsystem gyro) {
    this.gyro = gyro;

    rightOne.configFactoryDefault();
    rightTwo.configFactoryDefault();
    rightThree.configFactoryDefault();

    leftOne.configFactoryDefault();
    leftTwo.configFactoryDefault();
    leftThree.configFactoryDefault();

    rightTwo.follow(rightOne);
    rightThree.follow(rightOne);

    leftTwo.follow(leftOne);
    leftThree.follow(leftOne);

    rightOne.setInverted(InvertType.InvertMotorOutput);
    rightTwo.setInverted(InvertType.FollowMaster);
    rightThree.setInverted(InvertType.FollowMaster);

    rightOne.setSensorPhase(false);
    leftOne.setSensorPhase(false);
    rightTwo.setSensorPhase(false);
    leftTwo.setSensorPhase(false);

    TalonFXConfiguration leftConfig = getCommonDriveMotorConfig();
    TalonFXConfiguration rightConfig = getCommonDriveMotorConfig();
    TalonFXConfiguration leftEncConfig = getCommonDriveMotorConfig();
    TalonFXConfiguration rightEncConfig = getCommonDriveMotorConfig();

    leftEncConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
    rightEncConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

    rightConfig.diff0Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice(); //Local Integrated Sensor
    rightConfig.diff1Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice(); //Aux Selected Sensor
    rightConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorSum.toFeedbackDevice(); //Sum0 - Sum1

    rightConfig.primaryPID.selectedFeedbackCoefficient = 0.5;

    /* Configure the left Talon's selected sensor as integrated sensor */
    leftConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

    /* Configure the Remote (Left) Talon's selected sensor as a remote sensor for the right Talon */
    rightConfig.remoteFilter0.remoteSensorDeviceID = gyro.getCanId(); //Device ID of Remote Source
    rightConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor; //Remote Source Type
    
    /** Heading Configs */
    rightConfig.remoteFilter1.remoteSensorDeviceID = gyro.getCanId();    //Pigeon Device ID
    rightConfig.remoteFilter1.remoteSensorSource = RemoteSensorSource.Pigeon_Yaw; //This is for a Pigeon over CAN
    rightConfig.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.RemoteSensor1.toFeedbackDevice(); //Set as the Aux Sensor
    // TODO: Update based on Pigeon values.
    rightConfig.auxiliaryPID.selectedFeedbackCoefficient = 1; //3600.0 / 8192.0; //Convert Yaw to tenths of a degree
    //leftConfig.auxPIDPolarity = true;

    /**
     * 1ms per loop.  PID loop can be slowed down if need be.
     * For example,
     * - if sensor updates are too slow
     * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
     * - sensor movement is very slow causing the derivative error to be near zero.
     */
    int closedLoopTimeMs = 1;
    rightConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
    rightConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
    rightConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
    rightConfig.slot3.closedLoopPeriod = closedLoopTimeMs;
    
    //Configures the PIDF values for the position loop
    rightConfig.slot0.integralZone = Constants.DriveConstants.kSlot0Iz;
    rightConfig.slot0.kF = Constants.DriveConstants.kSlot0F;
    rightConfig.slot0.kP = Constants.DriveConstants.kSlot0P;
    rightConfig.slot0.kI = Constants.DriveConstants.kSlot0I;
    rightConfig.slot0.kD = Constants.DriveConstants.kSlot0D;
    //rightConfig.slot0.closedLoopPeakOutput = .5;

    //Configures the PIDF values for the heading loop
    rightConfig.slot1.integralZone = Constants.DriveConstants.kSlot1Iz;
    rightConfig.slot1.kF = Constants.DriveConstants.kSlot1F;
    rightConfig.slot1.kP = Constants.DriveConstants.kSlot1P;
    rightConfig.slot1.kI = Constants.DriveConstants.kSlot1I;
    rightConfig.slot1.kD = Constants.DriveConstants.kSlot1D;
    //rightConfig.slot1.closedLoopPeakOutput = .5;
    //rightConfig.slot1.closedLoopPeriod = 1;

    /* Set status frame periods to ensure we don't have stale data */
    // Dial down the feedback on all of the followers
    // rightOne.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
    // rightTwo.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
    // rightThree.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    // leftOne.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
    // rightTwo.setStatusFramePeriod(StatusFrame.Status_1_General, 20);
    // rightThree.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
    
    // leftOne.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
    // leftTwo.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
    // leftThree.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255);
    
    // Dial up the speed on everything we need.
    rightOne.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20);
    rightOne.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20);
    rightOne.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20);
    rightOne.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20);

    leftOne.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5);
    leftTwo.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5);
    rightTwo.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5);
    
    rightOne.changeMotionControlFramePeriod(5);


    configDriveMotor(rightOne, rightConfig);
    configDriveMotor(rightTwo, rightEncConfig);
    configDriveMotor(rightThree, getCommonDriveMotorConfig());
    
    configDriveMotor(leftOne, leftConfig);
    configDriveMotor(leftTwo, leftEncConfig);
    configDriveMotor(leftThree, getCommonDriveMotorConfig());

    rightOne.setSelectedSensorPosition(0);
    leftOne.setSelectedSensorPosition(0);

    setDriveMode(DriveMode.Normal);
  }

  public double getMotionProfilePosition() {
    return rightOne.getSelectedSensorPosition();
  }

  private void configDriveMotor(WPI_TalonFX motor, TalonFXConfiguration config) {
    motor.setNeutralMode(NeutralMode.Coast);
    motor.configAllSettings(config);
  }

  private TalonFXConfiguration getCommonDriveMotorConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.supplyCurrLimit.enable = true;
    config.supplyCurrLimit.triggerThresholdCurrent = 40;
    config.supplyCurrLimit.triggerThresholdTime = 40;
    config.supplyCurrLimit.currentLimit = 40;

    config.openloopRamp = .5;
    
    config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    
    config.motionProfileTrajectoryPeriod = (int)Constants.DriveConstants.kProfileResolution;
    config.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_5Ms;
    config.velocityMeasurementWindow = 10;
    config.peakOutputForward =  +1.0;
    config.peakOutputReverse =  -1.0;

    config.slot0.kP = DriveConstants.kSlot1P;
    config.slot0.kI = DriveConstants.kSlot1I;
    config.slot0.kD = DriveConstants.kSlot1D;
    config.slot0.kF = DriveConstants.kSlot1F;
    
    return config;
  }

  // --START Teleop Drive Functionality
  private final CheesyDriveHelper cheesyDriveHelper = new CheesyDriveHelper();

  public void drive(double throttle, double wheel) {
    boolean quickTurn = false;
    if(throttle <= .2) {
      quickTurn = true;
      wheel *= .75;
    }
    
    drive(throttle, wheel, quickTurn);
  }
  
  public void drive(double throttle, double wheel, boolean quickturn) {
    DriveSignal signal = cheesyDriveHelper.cheesyDrive(throttle, wheel, quickturn);
    
    //System.out.print(String.format("%1$f,%2$f,%3$f,%4$f", signal.rightMotor, signal.leftMotor, throttle, wheel));

    rightOne.set(ControlMode.PercentOutput, signal.rightMotor);
    leftOne.set(ControlMode.PercentOutput, signal.leftMotor);
  }
  // --END Teleop Drive Functionality

  public void setDriveMode(DriveMode mode) {
    switch(mode) {
      case Normal:
      case Disable:
        leftOne.set(ControlMode.PercentOutput, 0);
        leftTwo.follow(leftOne, FollowerType.PercentOutput);
        leftThree.follow(leftOne, FollowerType.PercentOutput);
        
        rightOne.set(ControlMode.PercentOutput, 0);
        rightTwo.follow(rightOne);
        rightThree.follow(rightOne);
        break;
      case MotionProfile:
        leftOne.follow(rightOne, FollowerType.AuxOutput1);
        leftTwo.follow(rightOne, FollowerType.AuxOutput1);
        leftThree.follow(rightOne, FollowerType.AuxOutput1);
        break;
    }
  }

  public void setNeutralMode(NeutralMode mode) {
    leftOne.setNeutralMode(mode);
    leftTwo.setNeutralMode(mode);
    leftThree.setNeutralMode(mode);
    rightOne.setNeutralMode(mode);
    rightTwo.setNeutralMode(mode);
    rightThree.setNeutralMode(mode);
  }

  public void update() {
    localizer.update();

    currentPose = localizer.getPoseEstimate();

    if(getControlMode() == ControlMode.MotionProfileArc) {
      rightOne.processMotionProfileBuffer();
    }
  }

  @Override
  public void periodic() {
    // TODO: Update this to be in it's own thread.
    debug();
  }

  private void debug() {
    if(currentPose != null) {
      SmartDashboard.putNumber("Drive:X", currentPose.getX());
      SmartDashboard.putNumber("Drive:Y", currentPose.getY());
      SmartDashboard.putNumber("Drive:Heading", currentPose.getHeading());
    }
    SmartDashboard.putNumber("Drive:RightEnc", rightOne.getSelectedSensorPosition());
    SmartDashboard.putNumber("Drive:LeftEnc", leftOne.getSelectedSensorPosition());

    //System.out.println(String.format("CurTargetPos:%1$f,CurTargetVel%2$f", rightOne.getActiveTrajectoryPosition(), rightOne.getActiveTrajectoryVelocity()));

    //System.out.print(String.format("%1$f,%2$f,%3$f,%4$f,%5$f,%6$f", rightOne.getMotorOutputPercent(), rightTwo.getMotorOutputPercent(), rightThree.getMotorOutputPercent(), leftOne.getMotorOutputPercent(), leftTwo.getMotorOutputPercent(), leftThree.getMotorOutputPercent()));
  }

  public double getHeading() {
    return Math.toRadians(gyro.getYaw());
  }

  public void setHeading(double heading) {

  }

  public void setPosition(Pose2d pose) {
    localizer.setPoseEstimate(pose);
    currentPose = localizer.getPoseEstimate();
  }
  
  public double getTrackWidth() {
    return kTrackWidth;
  }

  public List<Double> getWheelPositions() {
    return Arrays.asList(encToInches(leftOne.getSelectedSensorPosition()), encToInches(rightOne.getSelectedSensorPosition()));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static double encToInches(double enc) {
    return enc / kEncToInches;
  }

  public static double inchesToEnc(double inches) {
    return inches * kEncToInches;
  }

  public double getRightEnc() {
    return rightTwo.getSelectedSensorPosition();
  }

  public double getLeftEnc() {
    return leftTwo.getSelectedSensorPosition();
  }

  public void setupNewProfile() {
    rightOne.clearMotionProfileHasUnderrun();
    rightOne.clearMotionProfileTrajectories();
    trajStream.Clear();
    setDriveMode(DriveMode.Disable);
  }

  public void startProfile() {
    setDriveMode(DriveMode.MotionProfile);
    rightOne.startMotionProfile(trajStream, 50, ControlMode.MotionProfileArc);
  }

  public boolean isProfileComplete() {
    return rightOne.isMotionProfileFinished();
  }

  public TrajectoryVelocityConstraint getVelConstraint() {
    return kVelConstraint;
  }

  public TrajectoryAccelerationConstraint getAccelConstraint() {
    return kAccelConstraint;
  }

  public ControlMode getControlMode() {
    return rightOne.getControlMode();
  }

  public Pose2d getCurrentGlobalPosition(){
    return currentPose;
  }

  public TrajectoryBuilder getTrajectoryBuilder(boolean isReverse, double x, double y, double angle) {
    return getTrajectoryBuilder(isReverse, new Pose2d(x, y, Math.toRadians(angle)));
  }

  public TrajectoryBuilder getTrajectoryBuilder(boolean isReverse, Pose2d pose) {
    return new TrajectoryBuilder(pose, isReverse, kVelConstraint, kAccelConstraint);    
  }

  public TrajectoryBuilder getTrajectoryBuilder(double startTan, Pose2d pose) {
    return new TrajectoryBuilder(pose, startTan, kVelConstraint, kAccelConstraint);    
  }

  public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
    return new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(maxAngularVel),
            new TankVelocityConstraint(maxVel, trackWidth)
    ));
  }

  public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
    return new ProfileAccelerationConstraint(maxAccel);
  }

  public void resetEncoders() {
    rightOne.getSensorCollection().setIntegratedSensorPosition(0, 10);
    leftOne.setSelectedSensorPosition(0);
  }

  public void loadTrajectoryPoint(TrajectoryPoint point) {
    trajStream.Write(point);
  }
}
