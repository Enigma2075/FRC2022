// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import javax.sound.sampled.ReverbType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ArmPosition;
import frc.robot.subsystems.ClimberSubsystem.WinchPosition;

/** An example command that uses an example subsystem. */
public class WinchManual extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem climber;
  private final DoubleSupplier powerSupplier;
  private final boolean reverse;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public WinchManual(ClimberSubsystem climber, boolean reverse, DoubleSupplier powerSupplier) {
    this.climber = climber;
    this.powerSupplier = powerSupplier;
    this.reverse = reverse;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(climber.hasClimbStarted()) {
      double power = powerSupplier.getAsDouble() * .1;
      if(reverse) {
        climber.winchRaw(power * -1);
      }
      else {
        climber.winchRaw(power);
      }
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stopWinch();
  }
}
