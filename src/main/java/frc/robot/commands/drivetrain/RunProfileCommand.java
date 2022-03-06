package frc.robot.commands.drivetrain;

import java.util.ArrayList;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.TankKinematics;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.acmerobotics.roadrunner.profile.MotionProfile;

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

  private ArrayList<TrajectoryPoint> trajectoryPoints = new ArrayList<TrajectoryPoint>();

  private Pose2d curEndPose;

  private double curProfilePosOffset = 0;

  private final double halfTrackWidth;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RunProfileCommand(DriveSubsystem drivetrain, Pose2d startPose) {
    this.drivetrain = drivetrain;
    this.curEndPose = startPose;

    halfTrackWidth = drivetrain.getTrackWidth() / 2;

    addRequirements(drivetrain);
  }

  public void startProfile() {
    double posOffset = drivetrain.getMotionProfilePosition();
    for (TrajectoryPoint t : trajectoryPoints) {
      t.position += posOffset;

      //System.out.println(String.format("Pos:%1f,AuxPos:%2f,Vel:%2f,PosOffset:%2f", t.position, t.auxiliaryPos, t.velocity, posOffset));
 
      drivetrain.loadTrajectoryPoint(t);
    }

    drivetrain.startProfile();
  }
  
  public void addTrajectory(Trajectory trajectory) {
    addTrajectory(trajectory, false);
  }

  public void addTrajectory(Trajectory trajectory, boolean lastTraj) {
    loadTrajectory(trajectory, false, lastTraj);
  }

  public void addTrajectory(Trajectory trajectory, boolean reverseHeading, boolean lastTraj) {
    loadTrajectory(trajectory, reverseHeading, lastTraj);
  }

  public void addPointTurn(double angle, boolean reverseHeading, boolean lastAction) {
    MotionProfile profile = MotionProfileGenerator.generateSimpleMotionProfile(
      new MotionState(curEndPose.getHeading(), 0.0, 0.0, 0.0),
      new MotionState(curEndPose.getHeading() + angle, 0.0, 0.0, 0.0),
      Math.toRadians(500),
      Math.toRadians(500)
      );

    loadTurn(profile, reverseHeading, lastAction);
  }

  private void loadTurn(MotionProfile profile, boolean reverseHeading, boolean lastAction) {
    double profileFramesPerSec = (1000.0 / Constants.DriveConstants.kProfileResolution);

    int segmentCount = (int) (profile.duration() * profileFramesPerSec);

    // Loops through points based on the resolution of the profile and pushes them
    // to the talon's upper buffer
    for (int i = 0; i <= segmentCount; i++) {
      double timeInSec = ((double) i) / profileFramesPerSec;

      // All trajectory poses are in field relative
      var curState = profile.get(timeInSec);
      
      TrajectoryPoint point = new TrajectoryPoint();
      point.zeroPos = false;
      point.isLastPoint = false;
      point.profileSlotSelect0 = 0;
      point.profileSlotSelect1 = 1;
      point.useAuxPID = true;
      // The X velocity is the forward back on a TankDrive. The Y velocity would be a strafe.
      // We divide this by 10 to get it from sec to 100 ms.
      point.velocity = 0;
      point.position = curProfilePosOffset;

      double curHeading = curState.getX();

      if (Math.abs(Math.toDegrees(curHeading)) > 180 && reverseHeading) {
        point.auxiliaryPos = curHeading * (8192.0/(2.0 * Math.PI)) * -1;
      } else {
        point.auxiliaryPos = curHeading * (8192.0/(2.0 * Math.PI));
      }

      if (segmentCount == i) {
        if(curHeading < 0) {
          curHeading = (Math.PI * 2.0) + curHeading;
        }
        curEndPose = new Pose2d(curEndPose.vec(), curHeading);
        
        if (lastAction) {
          point.isLastPoint = true;
        }
      }

      trajectoryPoints.add(point);

      //System.out.println(String.format("Pos:%1f,AuxPos:%2f,Vel:%2f,Heading:%2f", point.position, point.auxiliaryPos, point.velocity, curHeading));
    }
  }

  private void loadTrajectory(Trajectory traj, boolean reverseHeading, boolean lastTraj) {
    double rightEncoder = 0;
    double leftEncoder = 0;

    double profileFramesPerSec = (1000.0 / Constants.DriveConstants.kProfileResolution);

    int segmentCount = (int) (traj.duration() * profileFramesPerSec);

    //Vector2d prevLeftVector = null;
    //Vector2d prevRightVector = null;

    // Loops through points based on the resolution of the profile and pushes them
    // to the talon's upper buffer
    for (int i = 0; i <= segmentCount; i++) {
      double timeInSec = ((double) i) / profileFramesPerSec;

      // All trajectory poses are in field relative
      var curPositionPose = traj.get(timeInSec);
      var velocity = traj.velocity(timeInSec);
      var curHeading = curPositionPose.getHeading();
      
      // Convert the velocity from field oriented to robot orientied.
      var curVelocity = Kinematics.fieldToRobotVelocity(curPositionPose, velocity);

      var wheelVelocities = TankKinematics.robotToWheelVelocities(curVelocity, drivetrain.getTrackWidth());

      TrajectoryPoint point = new TrajectoryPoint();
      point.zeroPos = false;
      point.isLastPoint = false;
      point.profileSlotSelect0 = 0;
      point.profileSlotSelect1 = 1;
      point.useAuxPID = true;
      // The X velocity is the forward back on a TankDrive. The Y velocity would be a strafe.
      // We divide this by 10 to get it from sec to 100 ms.
      point.velocity = DriveSubsystem.inchesToEnc(curVelocity.getX()) / 10.0;

      if (Math.abs(Math.toDegrees(curPositionPose.getHeading())) > 180 && reverseHeading) {
        point.auxiliaryPos = ((curHeading * -1) + (Math.PI * 2)) * (8192.0/(2.0 * Math.PI)) * -1;
      } else {
        point.auxiliaryPos = curHeading * (8192.0/(2.0 * Math.PI));
      }

      leftEncoder += wheelVelocities.get(0) * Constants.DriveConstants.kProfileResolution;
      rightEncoder += wheelVelocities.get(1) * Constants.DriveConstants.kProfileResolution;
        
      //if (i == 0) {
      //  prevLeftVector = generateNewVector(curPositionPose, false);
      //  prevRightVector = generateNewVector(curPositionPose, true);
      //} 
      //else {
        

      //  Vector2d curLeftVector = generateNewVector(curPositionPose, false);
      //  Vector2d curRightVector = generateNewVector(curPositionPose, true);

      //  double leftSegmentDistance = prevLeftVector.distTo(curLeftVector);
      //  double rightSegmentDistance = prevRightVector.distTo(curRightVector);

      //  rightEncoder = rightEncoder + (Math.signum(point.velocity) * DriveSubsystem.inchesToEnc(rightSegmentDistance));
      //  leftEncoder = leftEncoder + (Math.signum(point.velocity) * DriveSubsystem.inchesToEnc(leftSegmentDistance));

      //  prevLeftVector = curLeftVector;
      //  prevRightVector = curRightVector;
      //}

      point.position = ((rightEncoder + leftEncoder) / 2.0) + curProfilePosOffset;

      if (segmentCount == i) {
        curEndPose = curPositionPose;
        curProfilePosOffset  = point.position;

        if(lastTraj) {
          point.isLastPoint = true;
        }
      }
     
      //System.out.println(String.format("Pos:%1f,AuxPos:%2f,Vel:%2f,RightEnc:%2f,LeftEnc:%2f,Heading:%2f", point.position, point.auxiliaryPos, point.velocity, rightEncoder, leftEncoder, curPositionPose.getHeading()));

      trajectoryPoints.add(point);
    }

    System.out.println("Loaded");
  }

  public Pose2d getEndPose() {
      return curEndPose;
  }

  //private Vector2d generateNewVector(Pose2d pose, boolean right) {
  //  double direction = 1;
  //  if(right) {
  //    direction = -1;
  //  }

  //  double cos_angle = Math.cos(pose.getHeading() + (direction * (Math.PI / 2.0)));
  //  double sin_angle = Math.sin(pose.getHeading() + (direction * (Math.PI / 2.0)));

  //  double leftX = pose.getX() - halfTrackWidth * cos_angle;
  //  double leftY = pose.getY() - halfTrackWidth * sin_angle;

  //  return new Vector2d(leftX, leftY);
  //}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setupNewProfile();
    drivetrain.setNeutralMode(NeutralMode.Brake);
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
    //System.out.println(String.format("RightEnc:%2f,Left:%2f", drivetrain.getRightEnc(), drivetrain.getLeftEnc()));
    
    Pose2d errorPose = Kinematics.calculateFieldPoseError(curEndPose, drivetrain.getCurrentGlobalPosition());
    if(Math.abs(errorPose.getX()) < 2 && Math.abs(errorPose.getY()) < 2 && Math.abs(errorPose.getHeading()) < Math.toRadians(2)) {
      return true;
    }
    
    return drivetrain.isProfileComplete();
  }

  /**
   * Method that is invoked when isFinished() returns true or if the command is
   * interupted
   * Stops the drivetrain and the thread that is managing the talons' point buffer
   */
  @Override
  public void end(boolean interrupted) {
  }
}
