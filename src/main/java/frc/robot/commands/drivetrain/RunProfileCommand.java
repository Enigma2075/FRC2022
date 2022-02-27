package frc.robot.commands.drivetrain;

import java.util.ArrayList;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveMode;

/**
 * Command that given a RoadRunner Path object will generate a trajectory and
 * run the profile on the Drivetrain
 * The profiles are designed to use already built paths but generate
 * trajectories on the fly so that the path can be generated from the
 * position the robot is currently at on the field so that it can adjust for
 * previous errors
 */
public class RunProfileCommand extends CommandBase {
  protected final DriveSubsystem drivetrain;

  //private boolean isHolding = false; // A boolean value used to see if the profile is holding at its final position
                                     // used to check if the command can end

  private ArrayList<Trajectory> trajectories = new ArrayList<Trajectory>();

  private int loadedTrajectories = 0;

  private double rightEncoder = 0.0;
  private double leftEncoder = 0.0;

  private double halfTrackWidth;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunProfileCommand(DriveSubsystem drivetrain) {
    this.drivetrain = drivetrain;

    halfTrackWidth = drivetrain.getTrackWidth() / 2;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  public void startProfile() {
    for (Trajectory t : trajectories) {
      loadedTrajectories++;
      loadTrajectory(t);
    }

    drivetrain.startProfile();
  }

  private void loadTrajectory(Trajectory traj) {
    double profileFramesPerSec = (1000.0 / Constants.DriveConstants.kProfileResolution);

    int trajectorySegmentCount = (int) (traj.duration() * profileFramesPerSec);

    // System.out.println(String.format("number %1$d",trajectorySegmentCount));

    //Pose2d prevPositionPose = null;
    //Pose2d prevVelocityPose = null;
    Vector2d prevLeftVector = null;
    Vector2d prevRightVector = null;

    // Loops through points based on the resolution of the profile and pushes them
    // to the talon's upper buffer
    for (int i = 0; i <= trajectorySegmentCount; i++) {
      // System.out.println(String.format("count: %d", i));
      double timeInSec = ((double) i) / profileFramesPerSec;
      Pose2d curPositionPose = traj.get(timeInSec);
      Pose2d curVelocityPose = traj.velocity(timeInSec);

      TrajectoryPoint point = new TrajectoryPoint();
      point.zeroPos = false;
      point.isLastPoint = false;
      point.profileSlotSelect0 = 0;
      point.profileSlotSelect1 = 1;
      point.useAuxPID = true;
      // TODO: Not sure why we are dividing by 10
      point.velocity = DriveSubsystem.inchesToEnc(curVelocityPose.getX()) / 10.0;

      double curHeading = curPositionPose.getHeading();
      
      if (Math.toDegrees(curPositionPose.getHeading()) > 180.0) {
        // curHeading = curHeading - (2 * Math.PI);
        // TODO: Fix heading calc
        point.auxiliaryPos = Math.toDegrees(curHeading);
      } else {
        point.auxiliaryPos = Math.toDegrees(curHeading);
      }

      if (i == 0) {
        point.position = (rightEncoder + leftEncoder) / 2.0;
        prevLeftVector = generateNewVector(curPositionPose);
        prevRightVector = generateNewVector(curPositionPose);
      } else {
        Vector2d curLeftVector = generateNewVector(curPositionPose);
        Vector2d curRightVector = generateNewVector(curPositionPose);

        //System.out.println(String.format("%1$f,%2$f,%3$f,%4$f,%5$f,%6$f,%7$f", rightX, rightY, leftX, leftY,
        //    curPositionPose.getHeading(), curPositionPose.getX(), curPositionPose.getY()));

        // System.out.println(String.format("r(%1$f,%2$f),l(%3$f,%4$f)", rightX, rightY,
        // leftX, leftY));

        double leftSegmentDistance = prevLeftVector.distTo(curLeftVector);
        double rightSegmentDistance = prevRightVector.distTo(curRightVector);

        //double prevRightEncoder = rightEncoder;
        //double prevLeftEncoder = leftEncoder;

        rightEncoder = rightEncoder + (Math.signum(point.velocity) * DriveSubsystem.inchesToEnc(rightSegmentDistance));
        leftEncoder = leftEncoder + (Math.signum(point.velocity) * DriveSubsystem.inchesToEnc(leftSegmentDistance));

        prevLeftVector = curLeftVector;
        prevRightVector = curRightVector;

        point.position = (rightEncoder + leftEncoder) / 2.0;

        // point.velocity = Math.signum(point.velocity) * Math.abs((point.position -
        // ((prevLeftEncoder - prevRightEncoder) / 2.0)) * 10.0);

        // System.out.println(String.format("%1$f,%2$f,%3$f,%4$f", rightEncoder,
        // leftEncoder, point.position, point.velocity, point.auxiliaryPos));
      }
      if (trajectorySegmentCount == i && loadedTrajectories == trajectories.size())
        point.isLastPoint = true;

      //prevPositionPose = curPositionPose;
      //prevVelocityPose = curVelocityPose;

      System.out.println(String.format("Pos:%1f,AuxPos:%2f,RightEnc:%2f,Left:%2f", point.position, point.auxiliaryPos, rightEncoder, leftEncoder));
      //  System.out.println(point.auxiliaryPos);
      //  System.out.println(point.auxiliaryArbFeedFwd);
      //  System.out.println(String.format("Vel:%1$f zeroPos:%2$s last:%3$s pos:%4$f
      //  angPos:%5$f angVel:%6$f", point.velocity, Boolean.toString(point.zeroPos),
      //  Boolean.toString(point.isLastPoint), point.position, point.auxiliaryPos,
      // point.auxiliaryVel));

      drivetrain.loadTrajectoryPoint(point);
    }

    System.out.println("Loaded");
  }

  private Vector2d generateNewVector(Pose2d pose) {
    double cos_angle = Math.cos(pose.getHeading() - Math.PI / 2.0);
    double sin_angle = Math.sin(pose.getHeading() - Math.PI / 2.0);

    double leftX = pose.getX() - halfTrackWidth * cos_angle;
    double leftY = pose.getY() - halfTrackWidth * sin_angle;

    return new Vector2d(leftX, leftY);
  }

  public void addTrajectory(Trajectory trajectory) {
    this.trajectories.add(trajectory);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.trajectories.clear();
    loadedTrajectories = 0;

    drivetrain.setupNewProfile();
    drivetrain.setNeutralMode(NeutralMode.Brake);

    
    // TODO: This shouldn't be needed
    // double startPosition =
    // DriveSubsystem.inchesToEnc(drivetrain.getCurrentPosition());

    // TODO: Don't think this is needed anymore
    drivetrain.resetEncoders();

    //rightEncoder = drivetrain.getRightEnc();
    //leftEncoder = drivetrain.getLeftEnc();

    rightEncoder = 0;
    leftEncoder = 0;
  }

  public void clearTrajectories() {
    this.trajectories.clear();
    loadedTrajectories = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  /**
   * Method that defines if the command can end
   * currently only checks to see if the robot is holding at its final position
   */
  @Override
  public boolean isFinished() {
    System.out.println(String.format("RightEnc:%2f,Left:%2f", drivetrain.getRightEnc(), drivetrain.getLeftEnc()));
      
    return false;//drivetrain.isProfileComplete();
  }

  /**
   * Method that is invoked when isFinished() returns true or if the command is
   * interupted
   * Stops the drivetrain and the thread that is managing the talons' point buffer
   */
  @Override
  public void end(boolean interrupted) {
    drivetrain.setDriveMode(DriveMode.Normal);
    drivetrain.setNeutralMode(NeutralMode.Coast);
  }
}
