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
    Forwards(4300), // MAX
    Hold(0),
    Grab(3300),
    ForwardGrab(2300),
    ForwardLatch(2900),
    LetGo(1400),
    //MidFront(2),
    Middle(3300), // Straight Up
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

  //private static final double kOuterPivotOffset = 4096-4020;
  //private static final double kInnerPivotOffset = 710;
  private static final double kOuterPivotMaxGravityFF = 0;//.075
  private static final double kInnerPivotMaxGravityFF = 0;//.05
  private static final double kOuterPivotCruiseVelocity = 1200; // Measured max velocity 860
  private static final double kInnerPivotCruiseVelocity = 1200; // Measured max velocity 860
  private static final double kOuterPivotAccelerationVelocity = 1200;
  private static final double kInnerPivotAccelerationVelocity = 1200;

  
  //private final WPI_TalonSRX tapeMeasure;
  private final WPI_TalonSRX pivot;

  private final double pivotCruiseVelocity;
  private final double pivotAccelerationVelocity;

  private final Side side;
  
  /** Creates a new ExampleSubsystem. */
  public ClimberArmSubsystem(int pivotCanId, Side side) {
    this.side = side;

    //tapeMeasure = new WPI_TalonSRX(tapeMasureCanId);
    pivot = new WPI_TalonSRX(pivotCanId);

    //tapeMeasure.configFactoryDefault(10);
    pivot.configFactoryDefault(10);

    TalonSRXConfiguration tapeConfig = new TalonSRXConfiguration();
    tapeConfig.continuousCurrentLimit = 3;
    tapeConfig.peakCurrentLimit = 15;
    tapeConfig.peakCurrentDuration = 100;

    //tapeMeasure.configAllSettings(tapeConfig);
    //tapeMeasure.enableCurrentLimit(false);
    //tapeMeasure.setNeutralMode(NeutralMode.Coast);

    //SensorCollection sensors = pivot.getSensorCollection();
    
    double pivotPoistion = 0;

    if (this.side == Side.Outer) {
      //pivotPoistion = Math.abs((int)(4096.0 - sensors.getPulseWidthPosition() - kOuterPivotOffset));
      //pivot.setSelectedSensorPosition(Math.abs((int)(4096.0 - sensors.getPulseWidthPosition() - kOuterPivotOffset)), 0, 10);
      
      //this.pivotOffset = kOuterPivotOffset;    
      //this.pivotMaxGravityFF = kOuterPivotMaxGravityFF;
      this.pivotCruiseVelocity = kOuterPivotCruiseVelocity;
      this.pivotAccelerationVelocity = kOuterPivotAccelerationVelocity; 
    }
    else {
      pivot.setInverted(InvertType.InvertMotorOutput);
      //sensors.setQuadraturePosition((int)(sensors.getPulseWidthPosition() - kInnerPivotOffset), 10);
      //pivotPoistion = (int)(sensors.getPulseWidthPosition() - kInnerPivotOffset);
      
      //tapeMeasure.setInverted(InvertType.InvertMotorOutput);

      //this.pivotOffset = kInnerPivotOffset;
      //this.pivotMaxGravityFF = kInnerPivotMaxGravityFF;
      this.pivotCruiseVelocity = kInnerPivotCruiseVelocity;
      this.pivotAccelerationVelocity = kInnerPivotAccelerationVelocity; 
    }

    pivot.setNeutralMode(NeutralMode.Brake);

    TalonSRXConfiguration pivotConfig = new TalonSRXConfiguration();

    pivotConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;

    pivotConfig.motionCruiseVelocity = pivotCruiseVelocity;
    pivotConfig.motionAcceleration = pivotAccelerationVelocity;

    pivotConfig.slot0.kP = 1.5;

    pivot.configAllSettings(pivotConfig);

    pivot.setSelectedSensorPosition(pivotPoistion, 0, 10);
      
    //configServo(top, topPort);
    //configServo(bottom, bottomPort);
  }

  //private void configServo(Servo servo, int port) {
  //  servo.setBounds(2, 1.6, 1.5, 1.4, 1);
  //}

  //public void setTape(double power) {
  //  tapeMeasure.set(ControlMode.PercentOutput, power);
  //}

  public void holdPivot() {
    pivot.set(ControlMode.MotionMagic, pivot.getSelectedSensorPosition());
  }

  public void setPivotCoast() {
    pivot.setNeutralMode(NeutralMode.Coast);
    pivot.set(ControlMode.PercentOutput, 0);
    //pivot.set(ControlMode.PercentOutput, -0.2);
  }

  public void setPivot(PivotPosition position) {
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
    //SensorCollection sensors = pivot.getSensorCollection();
    //SmartDashboard.putNumber("Climber:Pivot:" + side + ":Absolute", sensors.getPulseWidthPosition());
   
    //SmartDashboard.putNumber("Climber:Pivot:" + side + ":Position", pivot.getSelectedSensorPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}