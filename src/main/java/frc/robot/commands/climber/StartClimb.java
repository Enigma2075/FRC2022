// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ArmPosition;
import frc.robot.subsystems.ClimberSubsystem.WinchPosition;

/** An example command that uses an example subsystem. */
public class StartClimb extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ClimberSubsystem climber;
  private final ShooterSubsystem shooter;
  private final static double kTurretAngle = 180;
  private final DoubleSupplier startClimb;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public StartClimb(ClimberSubsystem climber, ShooterSubsystem shooter, DoubleSupplier startClimb) {
    this.climber = climber;
    this.shooter = shooter;
    this.startClimb = startClimb;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(startClimb.getAsDouble() < 0) {
      return;
    }
    Pullup.finished = false;
    
    ClimberSubsystem.startClimb();
    
    // Move the turret so it is out of the way.
    shooter.turret(kTurretAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(startClimb.getAsDouble() < 0) {
      return;
    }
    if(Math.abs(shooter.getTurretAngle() - kTurretAngle) < 20) {
      climber.pivot(ArmPosition.InitialGrab, true);

      if(Math.abs(climber.getOuterPivotError()) < 1000) {
        climber.winch(WinchPosition.InitialGrab);
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
    if(startClimb.getAsDouble() < 0) {
      return;
    }
    climber.stop();
  }
}
