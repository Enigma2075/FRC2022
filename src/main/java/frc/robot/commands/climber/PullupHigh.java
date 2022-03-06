// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ArmPosition;
import frc.robot.subsystems.ClimberSubsystem.WinchPosition;

/** An example command that uses an example subsystem. */
public class PullupHigh extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem climber;
  private final BooleanSupplier liftOff;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PullupHigh(ClimberSubsystem climber, BooleanSupplier liftOff) {
    this.climber = climber;
    this.liftOff= liftOff;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.winch(WinchPosition.InnerOut, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  int count = 0;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(count < 8) {
      count ++;
    }
    else {
      climber.pivot(ArmPosition.Coast, true);
    }
    
    if(liftOff.getAsBoolean()) {
      //climber.pivot(ArmPosition.InnerLetGo, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }
}
