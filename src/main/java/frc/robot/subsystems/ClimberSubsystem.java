// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase { 
  private ClimberArmSubsystem right = new ClimberArmSubsystem(0, 1);
  private ClimberArmSubsystem left = new ClimberArmSubsystem(2, 3);

  /** Creates a new ExampleSubsystem. */
  public ClimberSubsystem() {
  }

  public void stop() {
    right.stop();
    left.stop();
  }

  public void out() {
    right.out();
    left.out();
  }

  public void in() {
    right.in();
    left.in();
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
