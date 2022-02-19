// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberArmSubsystem.PivotPosition;

public class ClimberSubsystem extends SubsystemBase { 
  private ClimberArmSubsystem inner = new ClimberArmSubsystem(ClimberConstants.kInnerTapeCanId, ClimberConstants.kInnerPivotCanId, false);
  private ClimberArmSubsystem outer = new ClimberArmSubsystem(ClimberConstants.kOuterTapeCanId, ClimberConstants.kOuterPivotCanId, true);

  private WPI_TalonFX winch = new WPI_TalonFX(ClimberConstants.kWinchCanId, "canivore");

  /** Creates a new ExampleSubsystem. */
  public ClimberSubsystem() {
  }

  public void stop() {
    inner.stop();
    outer.stop();
  }

  public void out(double power) {
    inner.out(power);
    outer.out(power);
  }

  public void in(double power) {
    inner.in(power);
    outer.in(power);
  }

  public void pivot(PivotPosition position) {
    inner.pivot(position);
    outer.pivot(position);
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
