// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.intakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  public enum IntakeState {
    Intake,
    Outtake,
    DownAndOff,
    PartialIntake,
    FullyUp,
    Stop
  }

  private IntakeState currentState = IntakeState.Stop;

  public enum PivotPosition {
    Down(1810),
    Up(-220),
    HelpIntake(1000),
    FullyUp(0);

    private int value;
    private static Map map = new HashMap<>();

    private PivotPosition(int value) {
      this.value = value;
    }

    static {
      for (PivotPosition pivotPosition : PivotPosition.values()) {
        map.put(pivotPosition.value, pivotPosition);
      }
    }

    public static PivotPosition valueOf(int pivotPosition) {
      return (PivotPosition) map.get(pivotPosition);
    }

    public int getValue() {
      return value;
    }
  }

  private final WPI_TalonFX barMotor = new WPI_TalonFX(intakeConstants.kBarCanId, Constants.GeneralConstants.kCanBusRioName);
  private final WPI_TalonSRX pivotMotor = new WPI_TalonSRX(intakeConstants.kPivotCanId);

  private static final double kPivotZeroOffset = 3756; // To get this value 

  private static final double kPivotMaxGravityFF = .1;
  private static final double kPivotCruiseVelocity = 800; // Measured max velocity 800
  private static final double kPivotAccelerationVelocity = 1500;
  private static final double kPivotP = 3;

  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    barMotor.configFactoryDefault(10);
    pivotMotor.configFactoryDefault(10);

    barMotor.setNeutralMode(NeutralMode.Coast);

    SensorCollection sensors = pivotMotor.getSensorCollection();
    sensors.setQuadraturePosition((int)(sensors.getPulseWidthPosition() - kPivotZeroOffset), 10);

    pivotMotor.setInverted(InvertType.InvertMotorOutput);
    pivotMotor.setNeutralMode(NeutralMode.Brake);

    TalonSRXConfiguration config = new TalonSRXConfiguration();

    config.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    config.motionCruiseVelocity = kPivotCruiseVelocity;
    config.motionAcceleration = kPivotAccelerationVelocity;

    config.slot0.kP = kPivotP;

    pivotMotor.configAllSettings(config, 10);
  }

  public void downAndOff() {
    if(currentState != IntakeState.DownAndOff) {
      barMotor.set(ControlMode.PercentOutput, 0);
      pivotTo(PivotPosition.Down);
      currentState = IntakeState.DownAndOff;
    }
  }

  public void intake() {
    if(currentState != IntakeState.Intake) {
      barMotor.set(ControlMode.PercentOutput, .80);
      pivotTo(PivotPosition.Down);
      currentState = IntakeState.Intake;
    }
  }

  public void fullyUp() {
    if(currentState != IntakeState.FullyUp) {
      barMotor.set(ControlMode.PercentOutput, 0);
      pivotTo(PivotPosition.FullyUp);
      currentState = IntakeState.FullyUp;
    }
  }

  public void outtake() {
    if(currentState != IntakeState.Outtake) {
      currentState = IntakeState.Outtake;
      barMotor.set(ControlMode.PercentOutput, -.70);
    }
  }

  public void helpIntake() {
    if(currentState != IntakeState.PartialIntake) {
      barMotor.set(ControlMode.PercentOutput, .80);
      pivotTo(PivotPosition.HelpIntake);
      currentState = IntakeState.PartialIntake;
    }
  }

  public void stop() {
    if(currentState != IntakeState.Stop) {
      barMotor.set(ControlMode.PercentOutput, 0.2);
      pivotTo(PivotPosition.Up);
      currentState = IntakeState.Stop;
    }
  }

  private void pivotTo(PivotPosition position) {
    double kMeasuredPosHorizontal = 840; //Position measured when arm is horizontal
    double kTicksPerDegree = PivotPosition.Down.value / 90.0; // The pivot moves 90 degrees
    double currentPos = pivotMotor.getSelectedSensorPosition();
    double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);
    
    double arbitraryFF = kPivotMaxGravityFF * cosineScalar;

    pivotMotor.set(ControlMode.MotionMagic, position.getValue(), DemandType.ArbitraryFeedForward, arbitraryFF);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //debug();
  }

  public void debug() {
    SensorCollection sensors = pivotMotor.getSensorCollection();
    
    SmartDashboard.putNumber("Intake:Pivot:Absolute", sensors.getPulseWidthPosition());
    SmartDashboard.putNumber("Intake:Pivot:Position", pivotMotor.getSelectedSensorPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
