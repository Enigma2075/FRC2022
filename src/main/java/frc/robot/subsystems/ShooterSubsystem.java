// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase { 
  public final CANSparkMax hoodMotor = new CANSparkMax(ShooterConstants.kHoodCanId, MotorType.kBrushless);
  public final CANSparkMax popperMotor = new CANSparkMax(ShooterConstants.kPopperCanId, MotorType.kBrushless);
  public final WPI_TalonFX turretMotor = new WPI_TalonFX(ShooterConstants.kTurretCanId, GeneralConstants.kCanBusName);
  public final WPI_TalonFX bottomMotor = new WPI_TalonFX(ShooterConstants.kBottomCanId, GeneralConstants.kCanBusName);
  public final WPI_TalonFX topMotor = new WPI_TalonFX(ShooterConstants.kTopCanId, GeneralConstants.kCanBusName);

  // Measured Max Velocity 19900;
  final int maxVel = 14900; 

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    // Reset all motors to factory default
    turretMotor.configFactoryDefault();
    bottomMotor.configFactoryDefault();
    topMotor.configFactoryDefault();
    hoodMotor.restoreFactoryDefaults(true);
    popperMotor.restoreFactoryDefaults(true);
    
    // Configure hood motor
    hoodMotor.setIdleMode(IdleMode.kBrake);

    // Configure popper motor
    popperMotor.setIdleMode(IdleMode.kCoast);

    // Configure shooter motors
    bottomMotor.setInverted(InvertType.InvertMotorOutput);
    topMotor.setInverted(InvertType.None);

    TalonFXConfiguration config = getShooterMotorConfig();

    configShooterMotor(topMotor, config);
    configShooterMotor(bottomMotor, config);
  }

  private void configShooterMotor(WPI_TalonFX motor, TalonFXConfiguration config) {
    motor.setNeutralMode(NeutralMode.Coast);
    motor.configAllSettings(config);
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

  public void shoot() {
    // Works at 20 foot and wrench under front of shooter
    //topMotor.set(TalonFXControlMode.Velocity, maxVel * .675);
    //bottomMotor.set(TalonFXControlMode.Velocity, maxVel * .675);
    
    topMotor.set(TalonFXControlMode.Velocity, maxVel * .675);
    bottomMotor.set(TalonFXControlMode.Velocity, maxVel * .675);
    
    //20 foot front spin
    //topMotor.set(TalonFXControlMode.Velocity, maxVel * 1);
    //bottomMotor.set(TalonFXControlMode.Velocity, maxVel * .465);

    popperMotor.set(.8);
  }

  public void stop() {
    bottomMotor.set(ControlMode.PercentOutput, 0);
    topMotor.set(ControlMode.PercentOutput, 0);

    popperMotor.set(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    debug();
  }

  public void debug() {
    writeMotorDebug("Top", topMotor);
    writeMotorDebug("Bottom", bottomMotor);
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
