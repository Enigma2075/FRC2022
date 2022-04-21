// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberArmSubsystem.PivotPosition;
import frc.robot.subsystems.ClimberSubsystem.ArmPosition;
import frc.robot.subsystems.ClimberSubsystem.WinchPosition;

/** An example command that uses an example subsystem. */
public class PullupTrav extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem climber;
  public enum State {
    Initialize,
    WinchToLetGo,
    CoastArms,
    CrossUnder,
    ExtendToLatch,
    LatchTrav
  }

  public static State currentState = State.Initialize;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PullupTrav(ClimberSubsystem climber) {
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
    
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public void reset() {
    currentState = State.Initialize;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!ClimberSubsystem.hasClimbStarted()) {
      return;
    }
    
    switch (currentState) {
      case Initialize:
        currentState = State.WinchToLetGo;
        climber.winch(WinchPosition.PullUpTrav);
      break;
       case WinchToLetGo:
       if(Math.abs(climber.getWinchError()) < 15000) {
          climber.winch(WinchPosition.LetGoTrav);  
          climber.pivot(ArmPosition.TravPullup, true);
          currentState = State.CoastArms;
        }
        break;
       case CoastArms:
        //if(Math.abs(climber.getWinchError()) < 15000) {
           //climber.pivot(ArmPosition.CrossUnderTrav, true);
           //currentState = State.CrossUnder;
        //}
       break;
      // case CrossUnder:
      //   if(Math.abs(climber.getWinchError()) < 15000) {
      //     climber.pivot(ArmPosition.TravGrab, true);
      //     currentState = State.ExtendToLatch;
      //   }
      // break;
      // case ExtendToLatch:
      //   if(Math.abs(climber.getOuterPivotError()) < 200) {
      //     //climber.pivot(ArmPosition.TravLatch, true);
      //     climber.winch(WinchPosition.PullUpHighFull);
      //     currentState = State.LatchTrav;
      //   }
      // break;
      // case LatchTrav:
      //   if(Math.abs(climber.getWinchError()) < 15000) {
      //     climber.pivot(ArmPosition.TravLatch, true);
      //   }
      // break;
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
