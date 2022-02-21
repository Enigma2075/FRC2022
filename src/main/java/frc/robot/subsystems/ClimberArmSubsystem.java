// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberArmSubsystem extends SubsystemBase { 
  public enum Side {
    Outer,
    Inner
  }
  
  public enum PivotPosition {
    Forwards(2400),//
    //MidFront(2),
    Middle(1700),//0
    //MidBack(4),//
    Backwards(500);///

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

  private static final double kOuterPivotOffset = 4096-4020;
  private static final double kInnerPivotOffset = 710;
  private static final double kOuterPivotMaxGravityFF = 0;//.075
  private static final double kInnerPivotMaxGravityFF = 0;//.05
  private static final double kOuterPivotCruiseVelocity = 700; // Measured max velocity 860
  private static final double kInnerPivotCruiseVelocity = 700; // Measured max velocity 860
  private static final double kOuterPivotAccelerationVelocity = 1400;
  private static final double kInnerPivotAccelerationVelocity = 1400;

  
  private final WPI_VictorSPX tapeMeasure;
  private final WPI_TalonSRX pivot;

  private final double pivotOffset;
  private final double pivotMaxGravityFF;
  private final double pivotCruiseVelocity;
  private final double pivotAccelerationVelocity;

  private final Side side;
  
  /** Creates a new ExampleSubsystem. */
  public ClimberArmSubsystem(int tapeMasureCanId, int pivotCanId, Side side) {
    this.side = side;

    tapeMeasure = new WPI_VictorSPX(tapeMasureCanId);
    pivot = new WPI_TalonSRX(pivotCanId);
    
    tapeMeasure.configFactoryDefault(10);
    pivot.configFactoryDefault(10);

    SensorCollection sensors = pivot.getSensorCollection();
    
    double pivotPoistion = 0;

    if (this.side == Side.Outer) {
      pivot.setInverted(InvertType.InvertMotorOutput);
      pivotPoistion = Math.abs((int)(4096.0 - sensors.getPulseWidthPosition() - kOuterPivotOffset));
      //pivot.setSelectedSensorPosition(Math.abs((int)(4096.0 - sensors.getPulseWidthPosition() - kOuterPivotOffset)), 0, 10);
      
      this.pivotOffset = kOuterPivotOffset;    
      this.pivotMaxGravityFF = kOuterPivotMaxGravityFF;
      this.pivotCruiseVelocity = kOuterPivotCruiseVelocity;
      this.pivotAccelerationVelocity = kOuterPivotAccelerationVelocity; 
    }
    else {
      //sensors.setQuadraturePosition((int)(sensors.getPulseWidthPosition() - kInnerPivotOffset), 10);
      pivotPoistion = (int)(sensors.getPulseWidthPosition() - kInnerPivotOffset);
      
      this.pivotOffset = kInnerPivotOffset;
      this.pivotMaxGravityFF = kInnerPivotMaxGravityFF;
      this.pivotCruiseVelocity = kInnerPivotCruiseVelocity;
      this.pivotAccelerationVelocity = kInnerPivotAccelerationVelocity; 
    }

    pivot.setNeutralMode(NeutralMode.Brake);

    TalonSRXConfiguration config = new TalonSRXConfiguration();

    config.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;

    config.motionCruiseVelocity = pivotCruiseVelocity;
    config.motionAcceleration = pivotAccelerationVelocity;

    config.slot0.kP = 1.5;

    pivot.configAllSettings(config);

    pivot.setSelectedSensorPosition(pivotPoistion, 0, 10);
      
    //configServo(top, topPort);
    //configServo(bottom, bottomPort);
  }

  //private void configServo(Servo servo, int port) {
  //  servo.setBounds(2, 1.6, 1.5, 1.4, 1);
  //}

  public void out(double power) {
    tapeMeasure.set(ControlMode.PercentOutput, power);
  }

  public void in(double power) {
    tapeMeasure.set(ControlMode.PercentOutput, -1 * power);
  }

  public void stop() {
    tapeMeasure.set(ControlMode.PercentOutput, 0);
  }

  public void pivot(PivotPosition position) {
    pivot.set(ControlMode.MotionMagic, position.value);
    
    //pivot.set(ControlMode.PercentOutput, 1);

    // TODO: Add logic to move motor to correct position
    //get PivotPosition
    //currentPosition = get MotorPosition
    //if currentPosition = PivotPosition
    //then return;
    //else if currrentposition != PivotPosititon
    //set currentposition = tryingtomovetoo
  }

  // public double calculatePivotTarget(double target) {
  //   return target + pivotOffset;
  // }

  // public double getPosition() {
  //   return pivot.getSelectedSensorPosition() - pivotOffset; 
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    debug();
  }

  public void debug() {
    SensorCollection sensors = pivot.getSensorCollection();
    SmartDashboard.putNumber("Climber:Pivot:" + side + ":Absolute", sensors.getPulseWidthPosition());
   
    SmartDashboard.putNumber("Climber:Pivot:" + side + ":Position", pivot.getSelectedSensorPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}