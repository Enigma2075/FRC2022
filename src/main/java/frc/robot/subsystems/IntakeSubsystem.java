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
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.intakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  public enum PivotPosition {
    Down(1642),
    Up(300),
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

  public final WPI_VictorSPX barMotor = new WPI_VictorSPX(intakeConstants.kBarCanId);
  public final WPI_TalonSRX pivotMotor = new WPI_TalonSRX(intakeConstants.kPivotCanId);

  public static final double kPivotZeroOffset = 3191;
  public static final double kPivotMaxGravityFF = .15;
  public static final double kPivotCruiseVelocity = 400; // Measured max velocity
  public static final double kPivotAccelerationVelocity = 400;

  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    barMotor.configFactoryDefault();
    pivotMotor.configFactoryDefault();

    barMotor.setInverted(InvertType.InvertMotorOutput);
    barMotor.setNeutralMode(NeutralMode.Coast);

    pivotMotor.setInverted(InvertType.InvertMotorOutput);
    pivotMotor.setNeutralMode(NeutralMode.Brake);

    SensorCollection sensors = pivotMotor.getSensorCollection();
    double absolutePosition = sensors.getPulseWidthPosition();

    TalonSRXConfiguration config = new TalonSRXConfiguration();

    // config.supplyCurrLimit.enable = true;
    // config.supplyCurrLimit.triggerThresholdCurrent = 50;
    // config.supplyCurrLimit.triggerThresholdTime = 50;
    // config.supplyCurrLimit.currentLimit = 40;

    config.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    config.motionCruiseVelocity = kPivotCruiseVelocity;
    config.motionAcceleration = kPivotCruiseVelocity;


    pivotMotor.setSelectedSensorPosition(absolutePosition);

    // config.slot0.kP = intakeConstants.kSlot1P;
    // config.slot0.kI = intakeConstants.kSlot1I;
    // config.slot0.kD = intakeConstants.kSlot1D;
    /// config.slot0.kF = intakeConstants.kSlot1F;
  }

  // private void configBarMotor(WPI_VictorSPX motor, VictorSPXConfiguration
  // config) {
  // motor.setNeutralMode(NeutralMode.Coast);
  // motor.configAllSettings(config);
  // }

  // private VictorSPXConfiguration getCommonIntakeMotorConfig() {
  // VictorSPXConfiguration config = new VictorSPXConfiguration();

  // config.supplyCurrLimit.enable = true;
  // config.supplyCurrLimit.triggerThresholdCurrent = 50;
  // config.supplyCurrLimit.triggerThresholdTime = 50;
  // config.supplyCurrLimit.currentLimit = 40;

  // config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

  // config.slot0.kP = intakeConstants.kSlot1P;
  // config.slot0.kI = intakeConstants.kSlot1I;
  // config.slot0.kD = intakeConstants.kSlot1D;
  /// config.slot0.kF = intakeConstants.kSlot1F;

  // return config;
  // }

  public void intake() {
    // barMotor.set(ControlMode.PercentOutput, 1.00);
    pivotMotor.set(ControlMode.PercentOutput, 1);
  }

  public void outtake() {
    barMotor.set(ControlMode.PercentOutput, -1.00);
  }

  public void stop() {
    barMotor.set(ControlMode.PercentOutput, 0);
    pivotMotor.set(ControlMode.PercentOutput, 0);

  }

  public void pivotTo(PivotPosition position) {
    double kMeasuredPosHorizontal = 840; //Position measured when arm is horizontal
    double kTicksPerDegree = PivotPosition.Down.value / 90.0; // The pivot moves 90 degrees
    double currentPos = getPivotSensorPosition();
    double degrees = (currentPos - kMeasuredPosHorizontal) / kTicksPerDegree;
    double radians = java.lang.Math.toRadians(degrees);
    double cosineScalar = java.lang.Math.cos(radians);
    
    double arbitraryFF = kPivotMaxGravityFF * cosineScalar;

    pivotMotor.set(ControlMode.MotionMagic, calculatePivotTarget(position.getValue()), DemandType.ArbitraryFeedForward, arbitraryFF);
  }

  private double calculatePivotTarget(double target) {
    return target + kPivotZeroOffset;
  }

  public double getPivotSensorPosition() {
    return pivotMotor.getSelectedSensorPosition() - kPivotZeroOffset;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    debug();
  }

  public void debug() {
    //SensorCollection sensors = pivotMotor.getSensorCollection();
    //SmartDashboard.putNumber("Intake:Pivot:Absolute", sensors.getPulseWidthPosition());
    //SmartDashboard.putNumber("Intake:Pivot:Relative", sensors.getQuadraturePosition());
    
    SmartDashboard.putNumber("Intake:Pivot:Position", getPivotSensorPosition());
  }
  /*
   * public void writeMotorDebug(String prefix, WPI_TalonFX motor) {
   * SmartDashboard.putNumber("Shooter:" + prefix + ":Velocity",
   * motor.getSelectedSensorVelocity());
   * SmartDashboard.putNumber("Shooter:" + prefix + ":Error",
   * motor.getClosedLoopError());
   * }
   */

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
