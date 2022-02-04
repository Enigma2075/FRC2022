// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

/** An example command that uses an example subsystem. */
public class ClimbCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem climber;
  private final DoubleSupplier out;
  private final DoubleSupplier in;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimbCommand(ClimberSubsystem climber, DoubleSupplier out, DoubleSupplier in) {
    this.climber = climber;
    this.out = out;
    this.in = in;

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
    if(out.getAsDouble() > .1) {
      climber.out(out.getAsDouble());
    }
    else if(in.getAsDouble() > .1) {
      climber.in(in.getAsDouble());
    }
    else {
      climber.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }
}
