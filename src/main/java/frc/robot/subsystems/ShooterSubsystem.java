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
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.external.Interpolable;
import frc.external.InterpolatingDouble;
import frc.external.InterpolatingTreeMap;
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
  // public final WPI_TalonFX bottomMotor = new WPI_TalonFX(ShooterConstants.kBottomCanId,
  //     GeneralConstants.kCanBusAltName);
  // public final WPI_TalonFX topMotor = new WPI_TalonFX(ShooterConstants.kTopCanId, GeneralConstants.kCanBusAltName);
  
  public final PWM limeLightRight = new PWM(ShooterConstants.kLimeLightRightPwmPort);
  public final PWM limeLightLeft = new PWM(ShooterConstants.kLimeLightLeftPwmPort);
  
  private final SparkMaxPIDController hoodPid;
  private final RelativeEncoder hoodEncoder;
  private double hoodOffset = 0;
  //private final RelativeEncoder hoodEncoder;

  // Measured Max Velocity 20300;
  private static final double maxVel = 16500;

  // Positive X Right
  // Positive Y Forward

  private static final double kHoodP = 0.01;
  private static final double kHoodI = 0;
  private static final double kHoodD = 0;
  private static final double kHoodIz = 0;
  private static final double kHoodFF = 0.0005;
  private static final int kHoodMagicSlot = 0;
  private static final double kHoodMaxVel = 5000;
  private static final double kHoodMinVel = 0;
  private static final double kHoodMaxAcc = 10000;
  private static final double kHoodAllowedError = 0;

  private static final double kTurretCountsPerRotation = 55.625 * 2048.0;
  private static final double kTurretCountsPerDegree = kTurretCountsPerRotation / 365.0;
  private static final double kTurretInitialCount = kTurretCountsPerDegree * 82.0;
  private static final double kTurretCruiseVelocity = 18000; // Measured max velocity 20890
  private static final double kTurretAccelerationVelocity = 60000;
  private static final double kTurretPID0P = .2;
  private static final double kTurretPID0I = 0;
  private static final double kTurretPID0D = 0;
  private static final double kTurretPID0F = 0;
  private static final double kTurretPID1P = 0.0055;
  private static final double kTurretPID1I = 0;
  private static final double kTurretPID1D = 0;
  private static final double kTurretPID1F = .0465;
  private static final double kTurretForwardLimit = 111000;
  private static final double kTurretReverseLimit = kTurretInitialCount + 1000;
  
  private static final double kMaxPWM = 1000; // Servo Max 2500
  private static final double kMinPWM = 500;

  private static final double kFarShotPWM = 0;
  private static final double kCloseShotPWM = 0;

  private static final double kLowAngle = 0;
  private static final double kHighAngle = 0;

  private static final InterpolatingTreeMap kPowerMap = new InterpolatingTreeMap<InterpolatingDouble,InterpolatingDouble>(4);

  private static boolean shooting = false;

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    // Setup power treemap
    //kPowerMap.put(new InterpolatingDouble(val), new InterpolatingDouble(val));
    //kPowerMap.put(new InterpolatingDouble(val), new InterpolatingDouble(val));
    
    // Reset all motors to factory default
    turretMotor.configFactoryDefault(10);
    bottomMotor.configFactoryDefault(10);
    topMotor.configFactoryDefault(10);
    hoodMotor.restoreFactoryDefaults(true);
    popperMotor.restoreFactoryDefaults(true);

    // Configure hood motor
    //hoodEncoder = hoodMotor.getAlternateEncoder(Type.kQuadrature, 8192);
    hoodMotor.setInverted(true);
    hoodEncoder = hoodMotor.getEncoder();
    hoodOffset = hoodEncoder.getPosition();
    hoodPid = hoodMotor.getPIDController();

    hoodPid.setP(kHoodP);
    hoodPid.setI(kHoodI);
    hoodPid.setD(kHoodD);
    hoodPid.setIZone(kHoodIz);
    hoodPid.setFF(kHoodFF);
    
    hoodPid.setSmartMotionMaxVelocity(kHoodMaxVel, kHoodMagicSlot);
    hoodPid.setSmartMotionMinOutputVelocity(kHoodMinVel, kHoodMagicSlot);
    hoodPid.setSmartMotionMaxAccel(kHoodMaxAcc, kHoodMagicSlot);
    hoodPid.setSmartMotionAllowedClosedLoopError(kHoodAllowedError, kHoodMagicSlot);
    hoodPid.setOutputRange(-1, 1);

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

    turretConfig.slot0.kP = kTurretPID0P;

    turretConfig.slot1.kF = kTurretPID1F;
    turretConfig.slot1.kP = kTurretPID1P;

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
    showVision(false);
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

    config.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_1Ms;
    config.velocityMeasurementWindow = 32;

    config.slot0.kP = ShooterConstants.kSlot0P;
    config.slot0.kI = ShooterConstants.kSlot0I;
    config.slot0.kD = ShooterConstants.kSlot0D;
    config.slot0.kF = ShooterConstants.kSlot0F;

    return config;
  }

  public void turret(double heading) {
    double target = kTurretCountsPerDegree * heading;
    
    if(heading < 20) {
      target = kTurretForwardLimit;
    }

    if(target > kTurretForwardLimit) {
      target = kTurretForwardLimit;
    }
    else if (target < kTurretReverseLimit) {
      target = kTurretReverseLimit;
    }

    turretMotor.selectProfileSlot(0, 0);
    turretMotor.set(ControlMode.MotionMagic, target);
  }

  public void showVision(boolean yes) {
    NetworkTable table = getLimeLightTable();

    NetworkTableEntry entry = table.getEntry("stream");
    
    if(yes) {
        entry.setNumber(1);
    }
    else {
        entry.setNumber(2);
    }
  }

  public static boolean isShooting() {
    return shooting;
  }

  public boolean isShooterAtSpeed(double vel) {
    double targetVelocity = vel * maxVel;
    boolean isTopMotorAtSpeed = Math.abs(topMotor.getSelectedSensorVelocity() - targetVelocity) < 100;
    boolean isBottomMotorAtSpeed = Math.abs(bottomMotor.getSelectedSensorVelocity() - targetVelocity) < 100;
    return isTopMotorAtSpeed && isBottomMotorAtSpeed;
  }

  public double getDistanceFromTarget() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    double ty = table.getEntry("ty").getDouble(0);

    //double limelightAngle = 42;
    //double targetHeight = 102.25;
    //double limelightHeight = 26.4325;
    double limelightAngle = 33.6;
    //double targetHeight = 103;
    double targetHeight = 104;
    double limelightHeight = 26.375;

    double currentDistance = ((targetHeight - limelightHeight) / Math.tan(Math.toRadians(limelightAngle + ty)));

    return currentDistance;
  }

  public double getTurretAngle() {
    return turretMotor.getSelectedSensorPosition() / kTurretCountsPerDegree;
  }

  public boolean shoot(boolean force) {
    double speed = .38;
    topMotor.set(TalonFXControlMode.Velocity, maxVel * speed);
    bottomMotor.set(TalonFXControlMode.Velocity, maxVel * speed);
    
    boolean atSpeed = isShooterAtSpeed(speed);
    
    shooting = true;

    if(atSpeed) {
      popperMotor.set(1);
      return true;
    }
    else {
      popperMotor.set(0);
      return false;
    }
  }

  public double getShooterCurrent() {
    return bottomMotor.getSupplyCurrent();
  }

  public boolean shoot(double speed) {
    topMotor.set(TalonFXControlMode.Velocity, maxVel * speed);
    bottomMotor.set(TalonFXControlMode.Velocity, maxVel * speed);
    
    boolean atSpeed = isShooterAtSpeed(speed);
    
    shooting = true;
    
    if(atSpeed) {
      popperMotor.set(1);
      return true;
    }
    else {
      popperMotor.set(0);
      return false;
    }
  }

  //public power getSeedForDistance(double distance) {
  //  return kPowerMap.getInterpolated(distance);
  //}
  
  public boolean spinUp() {
    boolean targetAquired = aquireTarget();

    double currentDistance = getDistanceFromTarget();
    //double slope = (.525 - .415)/(114.02 - 53.2);
    //double speed = slope * currentDistance + (.53 - (114.02 * slope));

    double maxDist = 177;
    double maxDistSpeed = .585;
    double minDist = 94;
    double minDistSpeed = .4765;

    double slope = (maxDistSpeed - minDistSpeed)/(maxDist - minDist);
    double speed = slope * currentDistance + (maxDistSpeed - (maxDist * slope));

    boolean validDistance = currentDistance <= maxDist && currentDistance >= minDist;

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
    boolean canShoot = validDistance && atSpeed && targetAquired;

    //canShoot = atSpeed;
    
    SmartDashboard.putBoolean("Shooter:AtSpeed", atSpeed);
    SmartDashboard.putBoolean("Shooter:ValidDistance", validDistance);
    SmartDashboard.putBoolean("Shooter:TargetAquired", targetAquired);

    return canShoot;
  }

  public boolean shoot() {
    shooting = true;

    boolean canShoot = spinUp();

    if (canShoot) {
      popperMotor.set(1);
    } else {
      popperMotor.set(0);
    }

    return canShoot;
  }

  public void setHood(){
    //hoodPid.setReference(1, ControlType.kSmartVelocity);
    //hoodMotor.set(1);
    hoodPid.setReference(hoodOffset + 5, ControlType.kSmartMotion);
  }

  private NetworkTable getLimeLightTable() {
    return NetworkTableInstance.getDefault().getTable("limelight");
  }

  public void setLEDs(boolean on) {
    NetworkTable table = getLimeLightTable();

    NetworkTableEntry entry = table.getEntry("ledMode");
    if (on) {
      entry.setNumber(3);
    } else if (!on) {
      entry.setNumber(1);
    }
  }

  private PIDController aquireTargetController = new PIDController(.015, 0, 0);

  public boolean aquireTarget() {
    setLEDs(true);

    NetworkTable table = getLimeLightTable();

    double tv = table.getEntry("tv").getDouble(0);
    double tx = table.getEntry("tx").getDouble(0);
    double ty = table.getEntry("ty").getDouble(0);
    double ta = table.getEntry("ta").getDouble(0);

    double error = -tx;
    //double output = aquireTargetController.calculate(error, 0);
    double output = 0;
    
    double kP = .0145;
    double kF = 0.026;
    
    if (tv == 0) {
      // if((turretTalon.getControlMode() == ControlMode.MotionMagic &&
      // Math.abs(getCurrentFRange() - turretTalon.getSelectedSensorPosition()) < 100)
      // || turretTalon.getSelectedSensorPosition() > Constants.TURRET_F_LIMIT - 500){
      // turretTalon.set(ControlMode.Velocity, -1600);
      // }
      if(turretMotor.getControlMode() == ControlMode.Velocity) {
        turretMotor.selectProfileSlot(1, 0);
        turretMotor.set(ControlMode.Velocity, 0);
      }

      return false;
    }

    output = (Math.abs(kP * error) + kF) * Math.signum(error);

    if (Math.abs(output) > 1) {
      output = Math.signum(output);
    }
    
    //if(Math.abs(error) < 5) {
    //  output = .0 * Math.signum(error);
    //}
    
    if(Math.abs(error) < .5) {
      output = 0;
    }

    turretMotor.selectProfileSlot(1, 0);
    turretMotor.set(ControlMode.Velocity, output * kTurretCruiseVelocity);
    
    //SmartDashboard.putNumber("Vision:output", output);
    //SmartDashboard.putNumber("Vision:tx", tx);
    //SmartDashboard.putNumber("Vision:error", error);
    //SmartDashboard.putNumber("Vision:vel", output * kTurretCruiseVelocity);

    if(Math.abs(tx) < 1.25) {
      return true;
    }
    else {
      return false;
    }

  }

  public void stop() {
    bottomMotor.set(ControlMode.PercentOutput, 0);
    topMotor.set(ControlMode.PercentOutput, 0);

    popperMotor.set(0);

    turretMotor.selectProfileSlot(0, 0);
    turretMotor.set(ControlMode.MotionMagic, turretMotor.getSelectedSensorPosition());

    shooting = false;

    setLEDs(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //debug();
  }

  public void debug() {
    writeMotorDebug("Top", topMotor);
    writeMotorDebug("Bottom", bottomMotor);
    SmartDashboard.putNumber("Shooter:Distance", getDistanceFromTarget());
    SmartDashboard.putNumber("Turret:Angle", getTurretAngle());
    SmartDashboard.putNumber("Shooter:Hood:Position", hoodEncoder.getPosition() - hoodOffset);
    SmartDashboard.putNumber("Shooter:Hood:Velocity", hoodEncoder.getVelocity());
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
