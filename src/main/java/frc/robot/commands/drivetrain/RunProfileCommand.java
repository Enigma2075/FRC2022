package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AutoDrivetrainSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Command that given a RoadRunner Path object will generate a trajectory and run the profile on the Drivetrain
 * The profiles are designed to use already built paths but generate trajectories on the fly so that the path can be generated from the
 * position the robot is currently at on the field so that it can adjust for previous errors
 */
public class RunProfileCommand extends CommandBase {
  private final AutoDrivetrainSubsystem drivetrain;

  /**
   * internal class used to push motion profile points from the top buffer of the Talons' memory to the
   * lower buffer to be processed currently runs every 5ms but should be half of each points time aka 10ms per point
   * 5ms buffer time
   */
  private class KeepMotionAlive implements Runnable {
    @Override
    public void run() {
        drivetrain.processBuffer();
    }
  }

  private Notifier notifier = new Notifier(new KeepMotionAlive()); // instance of the notifier so the thread can start and end with the command
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunProfileCommand(AutoDrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }
}
