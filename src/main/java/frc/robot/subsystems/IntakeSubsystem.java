// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.intakeConstants;

public class IntakeSubsystem extends SubsystemBase { 
  
  public final WPI_VictorSPX barMotor = new WPI_VictorSPX(intakeConstants.kBarCanId);
  public final WPI_TalonSRX pivotMotor = new WPI_TalonSRX(intakeConstants.kPivotCanId);
 
  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    barMotor.configFactoryDefault();
    pivotMotor.configFactoryDefault();
    
    barMotor.setInverted(InvertType.InvertMotorOutput);
    barMotor.setNeutralMode(NeutralMode.Coast);

    pivotMotor.setInverted(InvertType.InvertMotorOutput);
    pivotMotor.setNeutralMode(NeutralMode.Brake);
    
    TalonSRXConfiguration config = new TalonSRXConfiguration();
  
    //config.supplyCurrLimit.enable = true;
    //config.supplyCurrLimit.triggerThresholdCurrent = 50;
    //config.supplyCurrLimit.triggerThresholdTime = 50;
    //config.supplyCurrLimit.currentLimit = 40;
    
    config.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Absolute;
    
    //config.slot0.kP = intakeConstants.kSlot1P;
    //config.slot0.kI = intakeConstants.kSlot1I;
    //config.slot0.kD = intakeConstants.kSlot1D;
    ///config.slot0.kF = intakeConstants.kSlot1F;    
  }

  //private void configBarMotor(WPI_VictorSPX motor, VictorSPXConfiguration config) {
  //  motor.setNeutralMode(NeutralMode.Coast);
  //  motor.configAllSettings(config);
  //}

  //private VictorSPXConfiguration getCommonIntakeMotorConfig() {
  //  VictorSPXConfiguration config = new VictorSPXConfiguration();


    //config.supplyCurrLimit.enable = true;
    //config.supplyCurrLimit.triggerThresholdCurrent = 50;
    //config.supplyCurrLimit.triggerThresholdTime = 50;
    //config.supplyCurrLimit.currentLimit = 40;
    
    //config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    
    //config.slot0.kP = intakeConstants.kSlot1P;
    //config.slot0.kI = intakeConstants.kSlot1I;
    //config.slot0.kD = intakeConstants.kSlot1D;
    ///config.slot0.kF = intakeConstants.kSlot1F;
    
  //  return config;
  //}

  public void intake() {
    barMotor.set(ControlMode.PercentOutput, 1.00);
    pivotMotor.set(ControlMode.PercentOutput, .5);
  }

  public void outtake() {
    barMotor.set(ControlMode.PercentOutput, -1.00);   
  }

  public void stop() {
    barMotor.set(ControlMode.PercentOutput, 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    debug();
  }

  public void debug() {
    SmartDashboard.putNumber("Intake:Pivot", pivotMotor.getSelectedSensorPosition());
  }
/*
   public void writeMotorDebug(String prefix, WPI_TalonFX motor) {
    SmartDashboard.putNumber("Shooter:" + prefix + ":Velocity", motor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Shooter:" + prefix + ":Error", motor.getClosedLoopError());
  }*/

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
