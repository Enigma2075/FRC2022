// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.climber.PullupHigh.State;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ArmPosition;
import frc.robot.subsystems.ClimberSubsystem.WinchPosition;

/** An example command that uses an example subsystem. */
public class ClimbCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem climber;
  private final DoubleSupplier inSupplier;
  private final DoubleSupplier outSupplier;

  private boolean isHolding = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClimbCommand(ClimberSubsystem climber, DoubleSupplier inSupplier, DoubleSupplier outSupplier) {
    this.climber = climber;
    this.inSupplier = inSupplier;
    this.outSupplier = outSupplier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!ClimberSubsystem.hasClimbStarted()) {
      climber.pivot(ArmPosition.Hold);
      climber.stopWinch();
      isHolding = false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    double triggerMin = .4;
    double in = inSupplier.getAsDouble();
    double out = outSupplier.getAsDouble();

    if(!ClimberSubsystem.hasClimbStarted()) {
      if(Math.abs(climber.getWinchError()) > 5000 && !isHolding) {
        isHolding = true;
        climber.winch(WinchPosition.Hold);
      }
      else if (Math.abs(climber.getWinchError()) < 100 && isHolding) {
        isHolding = false;
        climber.stopWinch();
      }
      //climber.winch(WinchPosition.Hold);
    }
    else if(in > triggerMin || out > triggerMin) {
      double power = 0;
      double sign = 1;

      if(!Pullup.finished) {
        sign = -1;
      }

      if(in > triggerMin) {
        power = in * .3 * sign;
      }
      else if(out > triggerMin) {
        power = out * -.3 * sign;
      }
      
      if(power != 0) {
        climber.winchRaw(power);
      }
    }
    else {
      climber.stopWinch();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}
