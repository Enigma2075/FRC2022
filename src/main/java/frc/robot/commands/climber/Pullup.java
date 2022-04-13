// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ArmPosition;
import frc.robot.subsystems.ClimberSubsystem.WinchPosition;

/** An example command that uses an example subsystem. */
public class Pullup extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem climber;

  public static boolean finished = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Pullup(ClimberSubsystem climber) {
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!ClimberSubsystem.hasClimbStarted()) {
      return;
    }
    //climber.runTapes();
    climber.winch(WinchPosition.PullUp, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!ClimberSubsystem.hasClimbStarted()) {
      return;
    }
    
    if(Math.abs(climber.getWinchError()) < 10000) {
      climber.pivot(ArmPosition.LatchHigh, true);  
      finished = true;    
    }
    else {
      climber.pivot(ArmPosition.InitialGrab, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!ClimberSubsystem.hasClimbStarted()) {
      return;
    }
    
    climber.stop();
  }
}
