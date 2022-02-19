// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberArmSubsystem extends SubsystemBase { 
  public enum PivotPosition {
    Forwards(1),
    MidFront(2),
    Middle(3),
    MidBack(4),
    Backwards(5);

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

  private final WPI_VictorSPX tapeMeasure;
  private final WPI_TalonSRX pivot;
  
  /** Creates a new ExampleSubsystem. */
  public ClimberArmSubsystem(int tapeMasureCanId, int pivotCanId) {
    tapeMeasure = new WPI_VictorSPX(tapeMasureCanId);
    pivot = new WPI_TalonSRX(pivotCanId);

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
    // TODO: Add logic to move motor to correct position
    //get PivotPosition
    //currentPosition = get MotorPosition
    //if currentPosition = PivotPosition
    //then return;
    //else if currrentposition != PivotPosititon
    //set currentposition = tryingtomovetoo
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
