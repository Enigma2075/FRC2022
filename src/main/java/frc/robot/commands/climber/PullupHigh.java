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
public class PullupHigh extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem climber;
  public enum State {
    Initialize,
    WinchToLetGo,
    CoastArms
  }

  State currentState = State.Initialize;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public PullupHigh(ClimberSubsystem climber) {
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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

    switch (currentState) {
      case Initialize:
        currentState = State.WinchToLetGo;    
      break;
      case WinchToLetGo:
        climber.winch(WinchPosition.OuterLetGo);     
        if(WinchPosition.PullUp.getValue() + 20000 < climber.getWinchEnc() ) {
          climber.pivot(ArmPosition.High, true);
          currentState = State.CoastArms;
        }
      break;
      case CoastArms:
        if(Math.abs(climber.getWinchError()) < 10000) {
          //climber.pivot(ArmPosition.OuterLetGo);
        }
      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }
}
