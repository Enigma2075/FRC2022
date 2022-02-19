// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberArmSubsystem extends SubsystemBase { 
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
    //top.setSpeed(power);
    //bottom.setSpeed(power);
  }

  public void in(double power) {
    //top.setSpeed(-1 * power);
    //bottom.setSpeed(-1 * power);
  }

  public void stop() {
    //top.setSpeed(0);
    //bottom.setSpeed(0);
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
