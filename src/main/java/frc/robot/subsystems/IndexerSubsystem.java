// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
  public final WPI_TalonSRX indexerMotor = new WPI_TalonSRX(IndexerConstants.kIndexerCanId);
  public final WPI_VictorSPX singleMotor = new WPI_VictorSPX(IndexerConstants.kSingulizerCanId);

  private DigitalInput position1 = new DigitalInput(IndexerConstants.kSensor1DioPort);
  private DigitalInput position2 = new DigitalInput(IndexerConstants.kSensor2DioPort);
  private DigitalInput position3 = new DigitalInput(IndexerConstants.kSensor3DioPort);

  /** Creates a new ExampleSubsystem. */
  public IndexerSubsystem() {
    indexerMotor.configFactoryDefault();
    singleMotor.configFactoryDefault();
    indexerMotor.setInverted(InvertType.InvertMotorOutput);

    /*
     * TalonFXConfiguration config = getCommonIndexerMotorConfig();
     * 
     * configIndexerMotor(indexerMotor, config);
     * configIndexerMotor(singleMotor, config);
     */
  }

  private void configIndexerMotor(WPI_TalonFX motor, TalonFXConfiguration config) {
    motor.setNeutralMode(NeutralMode.Coast);
    motor.configAllSettings(config);
  }

  private TalonFXConfiguration getCommonIndexerMotorConfig() {
    TalonFXConfiguration config = new TalonFXConfiguration();

    config.supplyCurrLimit.enable = true;
    config.supplyCurrLimit.triggerThresholdCurrent = 50;
    config.supplyCurrLimit.triggerThresholdTime = 50;
    config.supplyCurrLimit.currentLimit = 40;

    config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

    config.slot0.kP = IndexerConstants.kSlot1P;
    config.slot0.kI = IndexerConstants.kSlot1I;
    config.slot0.kD = IndexerConstants.kSlot1D;
    config.slot0.kF = IndexerConstants.kSlot1F;

    return config;
  }

  public void index() {
    index(false);
  }

  public void index(boolean force) {
    // Force both motors to run if the caller is forcing.
    if (force) {
      indexerMotor.set(ControlMode.PercentOutput, .80);
      singleMotor.set(ControlMode.PercentOutput, .80);

      return;
    }

    // If we don't have a ball in any position then don't run
    if(!getPosition1() && !getPosition3() && !getPosition2()) {
      indexerMotor.set(ControlMode.PercentOutput, 0);
      singleMotor.set(ControlMode.PercentOutput, 0);

      return;
    }

    // We already have all cargo in indexer
    if (getPosition3() && getPosition2()) {
      indexerMotor.set(ControlMode.PercentOutput, 0);
      singleMotor.set(ControlMode.PercentOutput, 0);
    } 
    // We have cargo in position3 && position1 but not position2
    else if (getPosition3() && !getPosition2() && getPosition1()) {
      indexerMotor.set(ControlMode.PercentOutput, 0);
      singleMotor.set(ControlMode.PercentOutput, .80);
    } 
    // We have cargo only in position1
    else if (!getPosition3() && !getPosition2() && getPosition1()) {
      indexerMotor.set(ControlMode.PercentOutput, .80);
      singleMotor.set(ControlMode.PercentOutput, .80);
    }
    else if (getPosition2() && !getPosition3()) {
      indexerMotor.set(ControlMode.PercentOutput, .80);
      singleMotor.set(ControlMode.PercentOutput, .80);
    }
    else {
      indexerMotor.set(ControlMode.PercentOutput, 0);
      singleMotor.set(ControlMode.PercentOutput, 0);
    }

    /*
     * if intake running
     * then run singleMotor
     * if sensor1 = true
     * then stop singleMotor, run indexerMotor
     * if sensor2 = true
     * then stop indexerMotor
     */

  }

  public boolean getPosition1() {
    return !position1.get();
  }

  public boolean getPosition2() {
    return !position2.get();
  }

  public boolean getPosition3() {
    return !position3.get();
  }

  public void stop() {
    indexerMotor.set(ControlMode.PercentOutput, 0);
    singleMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //debug();
  }

  public void debug() {
    SmartDashboard.putBoolean("Indexer:Sensor1", getPosition1());
    SmartDashboard.putBoolean("Indexer:Sensor2", getPosition2());
    SmartDashboard.putBoolean("Indexer:Sensor3", getPosition3());
    
    // writeMotorDebug("Index", indexerMotor);
    // writeMotorDebug("Singulizer", singulizerMotor);
  }

  /*
   * public void writeMotorDebug(String prefix, WPI_TalonFX motor) {
   * SmartDashboard.putNumber("Shooter:" + prefix + ":Velocity",
   * motor.getSelectedSensorVelocity());
   * SmartDashboard.putNumber("Shooter:" + prefix + ":Error",
   * motor.getClosedLoopError());
   * }
   * 
   * @Override
   * public void simulationPeriodic() {
   * // This method will be called once per scheduler run during simulation
   * }
   */
}
