// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ArmPosition;
import frc.robot.subsystems.ClimberSubsystem.WinchPosition;

/** An example command that uses an example subsystem. */
public class MoveClimbCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem climber;
  private final ArmPosition armPosition;
  private final WinchPosition winchPosition;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveClimbCommand(ClimberSubsystem climber, ArmPosition armPosition, WinchPosition winchPosition) {
    this.climber = climber;
    this.armPosition = armPosition;
    this.winchPosition = winchPosition;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.startTapes();
    climber.pivot(armPosition);
    climber.winch(winchPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  @Override
  public boolean isFinished() {
    return climber.isFinished();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      climber.stop(true);
    }
    else {   
      climber.stop();
    }
  }
}
