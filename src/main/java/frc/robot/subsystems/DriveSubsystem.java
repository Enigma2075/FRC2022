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
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TankVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.external.CheesyDriveHelper;
import frc.external.DriveSignal;
import frc.external.TankLocalizer;
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

  private static final double kMaxVel = encToInches(14000.0) * 10.0;
  private static final double kMaxAngVel = Math.toRadians(90);
  private static final double kTrackWidth = 24.125;
  private static final double kMaxAccel = kMaxVel * .75;

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


    TalonFXConfiguration config = getCommonDriveMotorConfig();

    configDriveMotor(rightOne, config);
    configDriveMotor(rightTwo, config);
    configDriveMotor(rightThree, config);
    
    configDriveMotor(leftOne, config);
    configDriveMotor(leftTwo, config);
    configDriveMotor(leftThree, config);

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
    
    rightOne.set(ControlMode.PercentOutput, signal.rightMotor);
    leftOne.set(ControlMode.PercentOutput, signal.leftMotor);
  }
  // --END Teleop Drive Functionality

  public void setDriveMode(DriveMode mode) {
    switch(mode) {
      case Normal:
      case Disable:
        leftOne.set(ControlMode.PercentOutput, 0);
        leftTwo.follow(rightOne, FollowerType.PercentOutput);
        leftThree.follow(rightOne, FollowerType.PercentOutput);
        rightOne.set(ControlMode.PercentOutput, 0);
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

  public double getHeading() {
    return Math.toRadians(gyro.getYaw());
  }

  public void setHeading(double heading) {

  }

  public void setPosition(double x, double y) {
    localizer.setPoseEstimate(new Pose2d(x, y, getHeading()));
  }
  
  public double getTrackWidth() {
    return kTrackWidth;
  }

  public List<Double> getWheelPositions() {
    return Arrays.asList(leftOne.getSelectedSensorPosition(), rightOne.getSelectedSensorPosition());
  }

  private void debug() {
    SmartDashboard.putNumber("Drive:X", currentPose.getX());
    SmartDashboard.putNumber("Drive:Y", currentPose.getY());
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

  public void setupNewProfile() {
    rightOne.clearMotionProfileHasUnderrun();
    rightOne.clearMotionProfileTrajectories();
    setDriveMode(DriveMode.Disable);
  }

  public void startProfile() {
    rightOne.startMotionProfile(trajStream, 50, ControlMode.MotionProfileArc);
  }

  public boolean isProfileComplete() {
    return rightOne.isMotionProfileFinished();
  }

  public ControlMode getControlMode() {
    return rightOne.getControlMode();
  }

  public Pose2d getCurrentGlobalPosition(){
    return new Pose2d(currentPose.getX(), currentPose.getY(), currentPose.getHeading());
  }

  public TrajectoryBuilder getTrajectoryBuilder(boolean isReverse) {
    return getTrajectoryBuilder(isReverse, getCurrentGlobalPosition());
  }

  public TrajectoryBuilder getTrajectoryBuilder(boolean isReverse, double x, double y, double angle) {
    return getTrajectoryBuilder(isReverse, new Pose2d(x, y, Math.toRadians(angle)));
  }

  public TrajectoryBuilder getTrajectoryBuilder(boolean isReverse, Pose2d pose) {
    return new TrajectoryBuilder(pose, isReverse, kVelConstraint, kAccelConstraint);    
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

}
