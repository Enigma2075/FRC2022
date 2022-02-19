// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimbSubsystem extends SubsystemBase { 
  public final WPI_TalonFX Outer = new WPI_TalonFX(ClimberConstants.kOuterCanId);
  public final WPI_TalonFX Inner = new WPI_TalonFX(ClimberConstants.kInnerCanId);

  
  /** Creates a new ExampleSubsystem. */
  public ClimbSubsystem() {
    Outer.configFactoryDefault();
    Inner.configFactoryDefault();

    //    TalonFXConfiguration config = getCommonClimbMotorConfig();
    //    configClimbMotor(Outer, config);
    //    configClimbMotor(Inner, config);

  }

  private TalonFXConfiguration getCommonClimbMotorConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.supplyCurrLimit.enable = true;
    config.supplyCurrLimit.triggerThresholdCurrent = 40;
    config.supplyCurrLimit.triggerThresholdTime = 40;
    config.supplyCurrLimit.currentLimit = 40;
        
    config.slot0.kP = ClimberConstants.kSlot1P;
    config.slot0.kI = ClimberConstants.kSlot1I;
    config.slot0.kD = ClimberConstants.kSlot1D;
    config.slot0.kF = ClimberConstants.kSlot1F;
    
    return config;
  }

  public void stop() {
    Inner.set(ControlMode.PercentOutput, 0);
    Outer.set(ControlMode.PercentOutput, 0);
  }

  public void outerInInnerOut() {
    Outer.setInverted(InvertType.InvertMotorOutput);
    Inner.set(ControlMode.PercentOutput, .80);
    Outer.set(ControlMode.PercentOutput, .80);
  }

  public void innerInOuterOut() {
    Inner.setInverted(InvertType.InvertMotorOutput);
    Inner.set(ControlMode.PercentOutput, .80);
    Outer.set(ControlMode.PercentOutput, .80);
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
