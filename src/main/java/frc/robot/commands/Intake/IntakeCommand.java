// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

/** An example command that uses an example subsystem. */
public class IntakeCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private IntakeSubsystem intakeSubsystem;
  DoubleSupplier intake;
  DoubleSupplier outtake;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @return 
   */
  public IntakeCommand(IntakeSubsystem intakeSubsystem, DoubleSupplier intake, DoubleSupplier outtake) {
    this.intakeSubsystem = intakeSubsystem;
    this.intake = intake;
    this.outtake = outtake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    
    
    if(intake.getAsDouble() > .1) {
      intakeSubsystem.intake();
    }
    else if(outtake.getAsDouble() > .1) {
      intakeSubsystem.outtake();
    }
    else {
      intakeSubsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stop();
  }
}
