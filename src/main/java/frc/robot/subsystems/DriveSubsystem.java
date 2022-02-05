// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.external.CheesyDriveHelper;
import frc.external.DriveSignal;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase { 
  public final WPI_TalonFX rightOne = new WPI_TalonFX(DriveConstants.kRightOneCanId);
  public final WPI_TalonFX rightTwo = new WPI_TalonFX(DriveConstants.kRightTwoCanId);
  public final WPI_TalonFX rightThree = new WPI_TalonFX(DriveConstants.kRightThreeCanId);

  public final WPI_TalonFX leftOne = new WPI_TalonFX(DriveConstants.kLeftOneCanId);
  public final WPI_TalonFX leftTwo = new WPI_TalonFX(DriveConstants.kLeftTwoCanId);
  public final WPI_TalonFX leftThree = new WPI_TalonFX(DriveConstants.kLeftThreeCanId);

  public DriveSubsystem() {
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

    //config.supplyCurrLimit.enable = true;
    //config.supplyCurrLimit.triggerThresholdCurrent = 50;
    //config.supplyCurrLimit.triggerThresholdTime = 50;
    //config.supplyCurrLimit.currentLimit = 40;
    
    config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
    
    config.slot0.kP = DriveConstants.kSlot1P;
    config.slot0.kI = DriveConstants.kSlot1I;
    config.slot0.kD = DriveConstants.kSlot1D;
    config.slot0.kF = DriveConstants.kSlot1F;
    
    return config;
  }

  private final CheesyDriveHelper cheesyDriveHelper = new CheesyDriveHelper();
  public void drive(double throttle, double wheel) {
    DriveSignal signal = cheesyDriveHelper.cheesyDrive(throttle, wheel, false);
    
    rightOne.set(ControlMode.PercentOutput, signal.rightMotor);
    leftOne.set(ControlMode.PercentOutput, signal.leftMotor);
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
