// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
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
import edu.wpi.first.math.filter.LinearFilter;
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
  //private double hoodOffset = 0;
  //private final RelativeEncoder hoodEncoder;

  // Measured Max Velocity 20300;
  private static final double maxVel = 16500;

  // Positive X Right
  // Positive Y Forward

  private static final double kPipeOff = 0;
  private static final double kPipeWide = 1;
  private static final double kPipeZoom = 2;

  private static final double kHoodP = 0.000001;
  private static final double kHoodI = 0.02;
  private static final double kHoodD = 0;
  private static final double kHoodIz = 1;
  private static final double kHoodFF = 0.0001;
  private static final int kHoodMagicSlot = 0;
  private static final double kHoodMaxVel = 5000;
  private static final double kHoodMinVel = 0;
  private static final double kHoodMaxAcc = 30000;
  private static final double kHoodAllowedError = 1.5;

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

  private static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kSpeedMap = new InterpolatingTreeMap<InterpolatingDouble,InterpolatingDouble>();
  private static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kHoodMap = new InterpolatingTreeMap<InterpolatingDouble,InterpolatingDouble>();

  private static double kSafeZoneDistance = 0;
  private static double kSafeZoneSpeed = 0;
  private static double kSafeZoneHood = 0;
  private static double kWallDistance = 0;
  private static double kWallSpeed = 0;
  private static double kWallHood = 0;
  private static double kMaxDistance = 17*12;
  private static double kMinDistance = 90;

  private double currentTx = 0;
  private double currentTy = 0;

  //private LinearFilter txFilter = LinearFilter.movingAverage(3);
  private LinearFilter tyFilter = LinearFilter.movingAverage(3);

  private double currentTv = 0;

    //atSpeed = shooter.shoot(.610, 38); //17 ft/216 in
    //atSpeed = shooter.shoot(.545, 21); //13 ft/170 in
    //atSpeed = shooter.shoot(.490, 0); //9 ft/124 in
    //atSpeed = shooter.shoot(.45, 0); //6 ft/90 in

  public static double[][] kDistanceSpeedValues = {
    { 90, .45 },
    { 124, .49 },
    { 170, .545},
    { 216, .61}
  };

  static {
    for (double[] pair : kDistanceSpeedValues) {
      kSpeedMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
    }
    
    kWallSpeed = kSpeedMap
            .getInterpolated(new InterpolatingDouble(kWallDistance)).value;
    kSafeZoneSpeed = kSpeedMap
            .getInterpolated(new InterpolatingDouble(kSafeZoneDistance)).value;
  }

  public static double[][] kDistanceHoodValues = {
    { 90, 0.0 },
    { 124, 0.1 },
    { 170, 21},
    { 216, 38}
  };

  static {
    for (double[] pair : kDistanceHoodValues) {
      kHoodMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
    }
    
    kWallHood = kHoodMap
            .getInterpolated(new InterpolatingDouble(kWallDistance)).value;
    kSafeZoneHood = kHoodMap
            .getInterpolated(new InterpolatingDouble(kSafeZoneDistance)).value;
  }

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
    hoodMotor.restoreFactoryDefaults();
    popperMotor.restoreFactoryDefaults();

    // Configure hood motor
    //hoodEncoder = hoodMotor.getAlternateEncoder(Type.kQuadrature, 8192);
    hoodMotor.setInverted(false);
    hoodEncoder = hoodMotor.getEncoder();
    hoodEncoder.setPosition(0);
    //hoodOffset = hoodEncoder.getPosition();
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

    turretMotor.configAllSettings(turretConfig, 10);

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

    setVision(false);
  }

  private void configShooterMotor(WPI_TalonFX motor, TalonFXConfiguration config) {
    motor.setNeutralMode(NeutralMode.Coast);
    motor.configAllSettings(config, 10);
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

  public static boolean isShooting() {
    return shooting;
  }

  public boolean isShooterAtSpeed(double vel) {
    double targetVelocity = vel * maxVel;
    boolean isTopMotorAtSpeed = Math.abs(topMotor.getSelectedSensorVelocity() - targetVelocity) < 100;
    boolean isBottomMotorAtSpeed = Math.abs(bottomMotor.getSelectedSensorVelocity() - targetVelocity) < 100;
    return isTopMotorAtSpeed && isBottomMotorAtSpeed;
  }

  public boolean isHoodAtPosition(double targetPosition) {
    return Math.abs(hoodEncoder.getPosition() - targetPosition) < 1;
  }

  public double getDistanceFromTarget() {
    double limelightAngle = 25; //33.6;
    double targetHeight = 103; // 102 at West
    double limelightHeight = 39.6;

    double currentDistance = ((targetHeight - limelightHeight) / Math.tan(Math.toRadians(limelightAngle + currentTy)));

    //SmartDashboard.putNumber("Shooter:AtDistance", currentDistance);
    
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

  public boolean shoot(double shooterSpeed, double hoodPosition) {
    setHood(hoodPosition);
    topMotor.set(TalonFXControlMode.Velocity, maxVel * shooterSpeed);
    bottomMotor.set(TalonFXControlMode.Velocity, maxVel * shooterSpeed);
    
    boolean atSpeed = isShooterAtSpeed(shooterSpeed);
    
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
  
  public double getSpeedForDistance(double distance) {
    return kSpeedMap.getInterpolated(new InterpolatingDouble(distance)).value;
  }

  public double getHoodForDistance(double distance) {
    return kHoodMap.getInterpolated(new InterpolatingDouble(distance)).value;
  }

  public boolean spinUp() {
    return spinUp(true);
  }
  
  public boolean spinUp(boolean aquireTarget) {
    boolean targetAquired = true;
    
    if(aquireTarget) {
      targetAquired = aquireTarget();
    }

    double currentDistance = getDistanceFromTarget();
    double speed = getSpeedForDistance(currentDistance);
    double hoodPosition = getHoodForDistance(currentDistance);

    boolean validDistance = currentDistance <= kMaxDistance && currentDistance >= kMinDistance;

    // Works at 20 foot and wrench under front of shooter
    // topMotor.set(TalonFXControlMode.Velocity, maxVel * .675);
    // bottomMotor.set(TalonFXControlMode.Velocity, maxVel * .675);

    // topMotor.set(TalonFXControlMode.Velocity, 1650);
    // bottomMotor.set(TalonFXControlMode.Velocity, 1650);
    
    setHood(hoodPosition);

    topMotor.set(TalonFXControlMode.Velocity, maxVel * speed);
    bottomMotor.set(TalonFXControlMode.Velocity, maxVel * speed);
    
    // topMotor.set(ControlMode.PercentOutput, .8);
    // bottomMotor.set(ControlMode.PercentOutput, .8);

    // 20 foot front spin
    // topMotor.set(TalonFXControlMode.Velocity, maxVel * 1);
    // bottomMotor.set(TalonFXControlMode.Velocity, maxVel * .465);

    // return false;

    boolean atHoodPosition = isHoodAtPosition(hoodPosition);
    boolean atSpeed = isShooterAtSpeed(speed);
    boolean canShoot = validDistance && atSpeed && targetAquired && atHoodPosition && currentTv != 0;

    SmartDashboard.putBoolean("Shooter:AtSpeed", atSpeed);
    SmartDashboard.putBoolean("Shooter:ValidDistance", validDistance);
    SmartDashboard.putBoolean("Shooter:TargetAquired", targetAquired);
    SmartDashboard.putBoolean("Shooter:AtHoodPosition", atHoodPosition);

    return canShoot;
  }

  public boolean shootNoAquireTarget() {
    shooting = true;

    boolean canShoot = spinUp(false);

    if (canShoot) {
      popperMotor.set(1);
    } else {
      popperMotor.set(0);
    }

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

  public void setHood(double position){
    //hoodPid.setReference(1, ControlType.kSmartVelocity);
    //hoodMotor.set(1);
    hoodPid.setReference(position, ControlType.kSmartMotion);

    //SmartDashboard.putNumber("Shooter:HoodTarget", position);
  }

  private NetworkTable getLimeLightTable() {
    return NetworkTableInstance.getDefault().getTable("limelight");
  }

  public void setVision(boolean shooting) {
    NetworkTable table = getLimeLightTable();

    NetworkTableEntry pipe = table.getEntry("pipeline");
    NetworkTableEntry stream = table.getEntry("stream");

    if (shooting) {
      stream.setNumber(1);
      pipe.setNumber(kPipeWide);
    } else if (!shooting) {
      stream.setNumber(2);
      pipe.setNumber(kPipeOff);
    }
  }

  public void updateVisionData() {
    NetworkTable table = getLimeLightTable();

    double oldTv = currentTv;
    currentTv = table.getEntry("tv").getDouble(0);

    if(currentTv !=0 && oldTv == 0) {
      //txFilter.reset();
      tyFilter.reset();
    }

    if(currentTv != 0) { 
      currentTx = table.getEntry("tx").getDouble(0); //txFilter.calculate(table.getEntry("tx").getDouble(0));
      currentTy = tyFilter.calculate(table.getEntry("ty").getDouble(0));
    }
  }

  public boolean aquireTarget() {
    NetworkTable table = getLimeLightTable();
    NetworkTableEntry pipe = table.getEntry("pipeline");

    double tv = currentTv;
    double tx = currentTx;

    if(tv == 0) {
      pipe.setNumber(kPipeWide);
    }
    else if (Math.abs(tx) < 5) {
      pipe.setNumber(kPipeWide);
    }
    else {
      pipe.setNumber(kPipeWide);
    }
 
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
    setVision(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //debug();
    //updateVisionData();
    //SmartDashboard.putNumber("Shooter:Distance", getDistanceFromTarget());
    //SmartDashboard.putNumber("Shooter:Hood:Position", hoodEncoder.getPosition());
  }

  public void debug() {
    writeMotorDebug("Top", topMotor);
    writeMotorDebug("Bottom", bottomMotor);
    SmartDashboard.putNumber("Shooter:Distance", getDistanceFromTarget());
    SmartDashboard.putNumber("Turret:Angle", getTurretAngle());
    SmartDashboard.putNumber("Shooter:Hood:Position", hoodEncoder.getPosition());
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
