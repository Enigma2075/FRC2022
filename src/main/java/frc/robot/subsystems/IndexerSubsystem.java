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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
  public enum IndexerState {
    Stop,
    OnlySingulizer,
    Run
  }

  private IndexerState currentState = IndexerState.OnlySingulizer;

  public final WPI_TalonSRX indexerMotor = new WPI_TalonSRX(IndexerConstants.kIndexerCanId);
  public final WPI_VictorSPX singleMotor = new WPI_VictorSPX(IndexerConstants.kSingulizerCanId);
  // public final WPI_TalonSRX singleMotor = new
  // WPI_TalonSRX(IndexerConstants.kSingulizerCanId);

  private static boolean position1HasCargo = false;
  private static boolean position2HasCargo = false;
  private static boolean position3HasCargo = false;

  private DigitalInput position1 = new DigitalInput(IndexerConstants.kSensor1DioPort);
  private DigitalInput position2 = new DigitalInput(IndexerConstants.kSensor2DioPort);
  private DigitalInput position3 = new DigitalInput(IndexerConstants.kSensor3DioPort);

  private DigitalInput redCargo = new DigitalInput(IndexerConstants.kRedCargoDioPort);

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

  public static boolean hasCargo() {
    return position1HasCargo || position2HasCargo || position3HasCargo;
  }

  private void configIndexerMotor(WPI_TalonFX motor, TalonFXConfiguration config) {
    motor.setNeutralMode(NeutralMode.Coast);
    motor.configAllSettings(config, 10);
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

  private boolean curPosition1;
  private boolean curPosition2;
  private boolean curPosition3;
  private boolean startlastCargoAtPosition3 = false;

  private Boolean lastCargoAtPosition3 = null;
  private double lastCargoAtPosition3timestamp = 0;
  
  public Boolean isRedCargoAtPosition3() {
    return lastCargoAtPosition3;
  }

  public void index() {
    index(false);
  }

  public void index(boolean force) {
    // Force both motors to run if the caller is forcing.
    curPosition1 = getPosition1();
    curPosition2 = getPosition2();
    curPosition3 = getPosition3();

    if(!startlastCargoAtPosition3 && curPosition3) {
      startlastCargoAtPosition3 = true;
      boolean curRedCargo = redCargo.get();

      if(lastCargoAtPosition3 == null && !curRedCargo) {
        lastCargoAtPosition3 = false;
      }
    }
    else if(startlastCargoAtPosition3 && curPosition3) {
      boolean curRedCargo = redCargo.get();
      if(lastCargoAtPosition3 == null && !curRedCargo) {
        lastCargoAtPosition3 = false;
      }
    }
    else if (startlastCargoAtPosition3 && !curPosition3 && lastCargoAtPosition3timestamp == 0) {
      if(lastCargoAtPosition3 == null) {
        lastCargoAtPosition3 = true;
      }
      lastCargoAtPosition3timestamp = Timer.getFPGATimestamp();
    }
    else if (startlastCargoAtPosition3 && Timer.getFPGATimestamp() - lastCargoAtPosition3timestamp > .25) {
      lastCargoAtPosition3timestamp = 0;
      lastCargoAtPosition3 = null;
      startlastCargoAtPosition3 = false;
    }

    IndexerState requestedState = IndexerState.OnlySingulizer;

    // If we don't have a ball in any position then don't run
    if (!curPosition1 && !curPosition3 && !curPosition2) {
      requestedState = IndexerState.OnlySingulizer;
    }

    // We already have all cargo in indexer
    if (curPosition3 && curPosition2) {
      requestedState = IndexerState.Stop;
    }
    // We have cargo in position3 && position1 but not position2
    else if (curPosition3 && !curPosition2 && curPosition1) {
      requestedState = IndexerState.OnlySingulizer;
    }
    // We have cargo only in position1
    else if (!curPosition3 && !curPosition2 && curPosition1) {
      requestedState = IndexerState.Run;
    } else if (curPosition2 && !curPosition3) {
      requestedState = IndexerState.Run;
    } else {
      requestedState = IndexerState.OnlySingulizer;
    }

    if (force) {
      requestedState = IndexerState.Run;
    }

    if (requestedState == currentState) {
      return;
    }

    currentState = requestedState;
    switch (currentState) {
      case Run:
        indexerMotor.set(ControlMode.PercentOutput, .80);
        singleMotor.set(ControlMode.PercentOutput, 1);
        break;
      case Stop:
        indexerMotor.set(ControlMode.PercentOutput, 0);
        singleMotor.set(ControlMode.PercentOutput, 0);
        break;
      case OnlySingulizer:
      default:
        indexerMotor.set(ControlMode.PercentOutput, 0);
        singleMotor.set(ControlMode.PercentOutput, 1);
        break;
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
    position1HasCargo = !position1.get();
    return position1HasCargo;
  }

  public boolean getPosition2() {
    position2HasCargo = !position2.get();
    return position2HasCargo;
  }

  public boolean getPosition3() {
    position3HasCargo = !position3.get();
    return position3HasCargo;
  }

  public void stop() {
    if(currentState != IndexerState.Stop) {
      currentState = IndexerState.Stop;
      lastCargoAtPosition3timestamp = 0;
      indexerMotor.set(ControlMode.PercentOutput, 0);
      singleMotor.set(ControlMode.PercentOutput, 0);
    }
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
    //Boolean redCargo = isRedCargoAtPosition3();
    //if (redCargo != null) {
      SmartDashboard.putBoolean("Indexer:RedCargo", this.redCargo.get());
    //}
    //SmartDashboard.putString("Indexer:State", currentState.name());
    // if(isRedCargoAtPosition3() != null) {
    // SmartDashboard.putBoolean("Indexer:RedCargo", isRedCargoAtPosition3());
    // }

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
