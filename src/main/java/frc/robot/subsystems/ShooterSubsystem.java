// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  public enum LimelightPosition {
    Default(900),
    High(1000),
    Low(500);

    private int value;
    private static Map map = new HashMap<>();

    private LimelightPosition(int value) {
      this.value = value;
    }

    static {
      for (LimelightPosition position : LimelightPosition.values()) {
        map.put(position.value, position);
      }
    }

    public static LimelightPosition valueOf(int position) {
      return (LimelightPosition) map.get(position);
    }

    public int getValue() {
      return value;
    }
  }

  public final CANSparkMax hoodMotor = new CANSparkMax(ShooterConstants.kHoodCanId, MotorType.kBrushless);
  public final CANSparkMax popperMotor = new CANSparkMax(ShooterConstants.kPopperCanId, MotorType.kBrushless);
  public final WPI_TalonFX turretMotor = new WPI_TalonFX(ShooterConstants.kTurretCanId,
      GeneralConstants.kCanBusAltName);
  public final WPI_TalonFX bottomMotor = new WPI_TalonFX(ShooterConstants.kBottomCanId,
      GeneralConstants.kCanBusRioName);
  public final WPI_TalonFX topMotor = new WPI_TalonFX(ShooterConstants.kTopCanId, GeneralConstants.kCanBusRioName);
  
  public final PWM limeLightRight = new PWM(ShooterConstants.kLimeLightRightPwmPort);
  public final PWM limeLightLeft = new PWM(ShooterConstants.kLimeLightLeftPwmPort);

  public final DoubleSupplier yawSupplier;

  // Measured Max Velocity 20300;
  private static final double maxVel = 16500;

  // Positive X Right
  // Positive Y Forward

  private static final double kTurretCountsPerRotation = 55.625 * 2048.0;
  private static final double kTurretCountsPerDegree = kTurretCountsPerRotation / 365.0;
  private static final double kTurretInitialCount = kTurretCountsPerDegree * 82.0;
  private static final double kTurretCruiseVelocity = 18000; // Measured max velocity 20890
  private static final double kTurretAccelerationVelocity = 60000;
  private static final double kTurretP = .2;
  private static final double kTurretForwardLimit = 112000;
  private static final double kTurretReverseLimit = kTurretInitialCount + 1000;
  
  private static final double kMaxPWM = 1000; // Servo Max 2500
  private static final double kMinPWM = 500;

  private static final double kFarShotPWM = 0;
  private static final double kCloseShotPWM = 0;

  private static final double kLowAngle = 0;
  private static final double kHighAngle = 0;

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem(DoubleSupplier yawSupplier) {
    this.yawSupplier = yawSupplier;

    // Reset all motors to factory default
    turretMotor.configFactoryDefault(10);
    bottomMotor.configFactoryDefault(10);
    topMotor.configFactoryDefault(10);
    hoodMotor.restoreFactoryDefaults(true);
    popperMotor.restoreFactoryDefaults(true);

    // Configure turrt motor
    turretMotor.setInverted(InvertType.None);
    turretMotor.setSensorPhase(false);

    TalonFXConfiguration turretConfig = new TalonFXConfiguration();
    turretConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    turretConfig.motionCruiseVelocity = kTurretCruiseVelocity;
    turretConfig.motionAcceleration = kTurretAccelerationVelocity;

    turretConfig.forwardSoftLimitEnable = true;
    turretConfig.forwardSoftLimitThreshold = kTurretForwardLimit;
    turretConfig.reverseSoftLimitEnable = true;
    turretConfig.reverseSoftLimitThreshold = kTurretReverseLimit;

    turretConfig.slot0.kP = kTurretP;

    turretMotor.configAllSettings(turretConfig);

    turretMotor.setSelectedSensorPosition(kTurretInitialCount);

    // Configure hood motor
    hoodMotor.setIdleMode(IdleMode.kBrake);

    // Configure popper motor
    popperMotor.setIdleMode(IdleMode.kCoast);

    // Configure shooter motors
    bottomMotor.setInverted(InvertType.InvertMotorOutput);
    topMotor.setInverted(InvertType.InvertMotorOutput);

    TalonFXConfiguration config = getShooterMotorConfig();

    configShooterMotor(topMotor, config);
    configShooterMotor(bottomMotor, config);

    // limeLightLeft.setBounds(2.5, deadbandMax, center, deadbandMin, .5);
    // limeLightRight.setBounds(2.5, deadbandMax, center, deadbandMin, .5);
    // limeLightLeft.setBounds(2, 1.599, 1.5, 1.499, 1.25);
    // limeLightRight.setBounds(kMaxPWM, 1501, 1500, 1499, kMinPWM);

    limeLightLeft.setDisabled();
    // limeLightRight.set(0);

    limeLightRight.enableDeadbandElimination(true);

    moveLimeLight(LimelightPosition.Default);

    setLEDs(false);
  }

  private void configShooterMotor(WPI_TalonFX motor, TalonFXConfiguration config) {
    motor.setNeutralMode(NeutralMode.Coast);
    motor.configAllSettings(config);
  }

  private void moveLimeLight(LimelightPosition position) {
    limeLightRight.setRaw(position.value);
  }

  private TalonFXConfiguration getShooterMotorConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.supplyCurrLimit.enable = true;
    config.supplyCurrLimit.triggerThresholdCurrent = 50;
    config.supplyCurrLimit.triggerThresholdTime = 50;
    config.supplyCurrLimit.currentLimit = 40;

    config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

    config.slot0.kP = ShooterConstants.kSlot1P;
    config.slot0.kI = ShooterConstants.kSlot1I;
    config.slot0.kD = ShooterConstants.kSlot1D;
    config.slot0.kF = ShooterConstants.kSlot1F;

    return config;
  }

  public void turret(double heading) {
    
    //double target = kTurretCountsPerDegree * heading;
    double target = kTurretCountsPerDegree * yawSupplier.getAsDouble();

    if(target == 0) {
      target = kTurretForwardLimit;
    }

    if(target > kTurretForwardLimit) {
      target = kTurretForwardLimit;
    }
    else if (target < kTurretReverseLimit) {
      target = kTurretReverseLimit;
    }

    turretMotor.set(ControlMode.MotionMagic, target);
  }

  public boolean isShooterAtSpeed(double vel) {
    double targetVelocity = vel * maxVel;
    boolean isTopMotorAtSpeed = Math.abs(topMotor.getSelectedSensorVelocity() - targetVelocity) < 200;
    boolean isBottomMotorAtSpeed = Math.abs(bottomMotor.getSelectedSensorVelocity() - targetVelocity) < 200;
    return isTopMotorAtSpeed && isBottomMotorAtSpeed;
  }

  public double getDistanceFromTarget() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double ty = table.getEntry("ty").getDouble(0);

    // Angle = 52 From Vertical
    // Height = 26.4325

    double currentDistance = ((102.25 - 26.4325) / Math.tan(Math.toRadians(90 - 52 + ty)));

    return currentDistance;
  }

  public boolean shoot(double speed) {
    // Works at 20 foot and wrench under front of shooter
    // topMotor.set(TalonFXControlMode.Velocity, maxVel * .675);
    // bottomMotor.set(TalonFXControlMode.Velocity, maxVel * .675);

    // topMotor.set(TalonFXControlMode.Velocity, 1650);
    // bottomMotor.set(TalonFXControlMode.Velocity, 1650);
    topMotor.set(TalonFXControlMode.Velocity, maxVel * speed);
    bottomMotor.set(TalonFXControlMode.Velocity, maxVel * speed);
    // topMotor.set(ControlMode.PercentOutput, .8);
    // bottomMotor.set(ControlMode.PercentOutput, .8);

    // 20 foot front spin
    // topMotor.set(TalonFXControlMode.Velocity, maxVel * 1);
    // bottomMotor.set(TalonFXControlMode.Velocity, maxVel * .465);

    // return false;

    boolean atSpeed = isShooterAtSpeed(speed);
    if (atSpeed) {
      popperMotor.set(1);
    } else {
      popperMotor.set(0);
    }

    return atSpeed;
  }

  public NetworkTable getLimeLightTable() {
    return NetworkTableInstance.getDefault().getTable("limelight");
  }

  public void setLEDs(boolean on) {
    NetworkTable table = getLimeLightTable();

    NetworkTableEntry entry = table.getEntry("ledMode");
    if (on) {
      entry.setNumber(0);
    } else if (!on) {
      entry.setNumber(1);
    }
  }

  public void aquireTarget() {
    // if(!isWithinRange()) {
    // setLEDs(false);
    // return;
    // }

    setLEDs(true);

    NetworkTable table = getLimeLightTable();

    double tv = table.getEntry("tv").getDouble(0);
    double tx = table.getEntry("tx").getDouble(0);
    double ty = table.getEntry("ty").getDouble(0);
    double ta = table.getEntry("ta").getDouble(0);

    double error = -tx;
    double output = 0.0;

    double kP = .045;
    double kF = .015;
    
    if (tv == 0) {
      // if((turretTalon.getControlMode() == ControlMode.MotionMagic &&
      // Math.abs(getCurrentFRange() - turretTalon.getSelectedSensorPosition()) < 100)
      // || turretTalon.getSelectedSensorPosition() > Constants.TURRET_F_LIMIT - 500){
      // turretTalon.set(ControlMode.Velocity, -1600);
      // }
      return;
    }

    // if(error < 2) {
    // table.getEntry("pipeline").setNumber(1);
    // }

    if (tx < 1.0) {
      output = kP * error + kF;
    } else if (tx > 1.0) {
      output = kP * error - kF;
    }

    if (Math.abs(output) > 1) {
      output = Math.signum(output);
    }

    turretMotor.set(ControlMode.PercentOutput, output);

    //SmartDashboard.putNumber("Vision:output", output);
    //SmartDashboard.putNumber("Vision:tx", tx);
    //SmartDashboard.putNumber("Vision:error", error);
  }

  public void stop() {
    bottomMotor.set(ControlMode.PercentOutput, 0);
    topMotor.set(ControlMode.PercentOutput, 0);

    popperMotor.set(0);

    turretMotor.set(ControlMode.MotionMagic, turretMotor.getSelectedSensorPosition());

    setLEDs(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    debug();
  }

  public void debug() {
    writeMotorDebug("Top", topMotor);
    writeMotorDebug("Bottom", bottomMotor);
    SmartDashboard.putNumber("Shooter:Distance", getDistanceFromTarget());
  }

  public void writeMotorDebug(String prefix, WPI_TalonFX motor) {
    SmartDashboard.putNumber("Shooter:" + prefix + ":Velocity", motor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Shooter:" + prefix + ":Error", motor.getClosedLoopError());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
