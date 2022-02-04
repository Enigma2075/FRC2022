// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase { 
  private ClimberArmSubsystem right = new ClimberArmSubsystem(ClimberConstants.kRightTopPwmChannel, ClimberConstants.kRightBottomPwmChannel);
  private ClimberArmSubsystem left = new ClimberArmSubsystem(ClimberConstants.kLeftTopPwmChannel, ClimberConstants.kLeftBottomPwmChannel);

  /** Creates a new ExampleSubsystem. */
  public ClimberSubsystem() {
  }

  public void stop() {
    right.stop();
    left.stop();
  }

  public void out(double power) {
    right.out(power);
    left.out(power);
  }

  public void in(double power) {
    right.in(power);
    left.in(power);
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
