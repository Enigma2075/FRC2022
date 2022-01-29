// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase { 
  public final WPI_TalonSRX popperMotor = new WPI_TalonSRX(8);
  public final WPI_TalonFX rightMotor = new WPI_TalonFX(15);
  public final WPI_TalonFX leftMotor = new WPI_TalonFX(14);
  public final WPI_TalonFX topMotor = new WPI_TalonFX(20);

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    rightMotor.configFactoryDefault();
    leftMotor.configFactoryDefault();
    topMotor.configFactoryDefault();

    popperMotor.configFactoryDefault();
    
    rightMotor.follow(leftMotor);
    rightMotor.setInverted(InvertType.OpposeMaster);

    TalonFXConfiguration config = getCommonShooterMotorConfig();

    configShooterMotor(topMotor, config);
    configShooterMotor(rightMotor, config);
    configShooterMotor(leftMotor, config);
  }

  private void configShooterMotor(WPI_TalonFX motor, TalonFXConfiguration config) {
    motor.setNeutralMode(NeutralMode.Coast);
    motor.configAllSettings(config);
  }

  private TalonFXConfiguration getCommonShooterMotorConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 40, 50, 50);
    //config.openloopRamp = 1;

    return config;
  }

  public void shoot() {
//Ticks/100ms 19900

    //rightMotor.set(ControlMode.PercentOutput, .45);
    //leftMotor.set(ControlMode.PercentOutput, .45);
    //topMotor.set(ControlMode.PercentOutput, .95);
    rightMotor.set(ControlMode.PercentOutput, .95);
    leftMotor.set(ControlMode.PercentOutput, .95);
    topMotor.set(ControlMode.PercentOutput, -.1);

    //rightMotor.set(ControlMode.PercentOutput, .40);
    //leftMotor.set(ControlMode.PercentOutput, .40);
    //topMotor.set(ControlMode.PercentOutput, .40);

    popperMotor.set(ControlMode.PercentOutput, .80);
  }

  public void stop() {
    rightMotor.set(ControlMode.PercentOutput, 0);
    leftMotor.set(ControlMode.PercentOutput, 0);
    topMotor.set(ControlMode.PercentOutput, 0);

    popperMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
