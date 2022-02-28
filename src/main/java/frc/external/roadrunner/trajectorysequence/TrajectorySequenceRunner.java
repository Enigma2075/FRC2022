package frc.external.roadrunner.trajectorysequence;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
import com.acmerobotics.roadrunner.util.NanoClock;

import frc.external.roadrunner.trajectorysequence.sequencesegment.SequenceSegment;
import frc.external.roadrunner.trajectorysequence.sequencesegment.TrajectorySegment;
import frc.external.roadrunner.trajectorysequence.sequencesegment.TurnSegment;
import frc.external.roadrunner.trajectorysequence.sequencesegment.WaitSegment;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

public class TrajectorySequenceRunner {
    public static String COLOR_INACTIVE_TRAJECTORY = "#4caf507a";
    public static String COLOR_INACTIVE_TURN = "#7c4dff7a";
    public static String COLOR_INACTIVE_WAIT = "#dd2c007a";

    public static String COLOR_ACTIVE_TRAJECTORY = "#4CAF50";
    public static String COLOR_ACTIVE_TURN = "#7c4dff";
    public static String COLOR_ACTIVE_WAIT = "#dd2c00";

    public static int POSE_HISTORY_LIMIT = 100;

    private final TrajectoryFollower follower;

    private final PIDFController turnController;

    private final NanoClock clock;

    private TrajectorySequence currentTrajectorySequence;
    private double currentSegmentStartTime;
    private int currentSegmentIndex;
    private int lastSegmentIndex;

    private Pose2d lastPoseError = new Pose2d();

    List<TrajectoryMarker> remainingMarkers = new ArrayList<>();

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    public TrajectorySequenceRunner(TrajectoryFollower follower, PIDCoefficients headingPIDCoefficients) {
        this.follower = follower;

        turnController = new PIDFController(headingPIDCoefficients);
        turnController.setInputBounds(0, 2 * Math.PI);

        clock = NanoClock.system();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        currentTrajectorySequence = trajectorySequence;
        currentSegmentStartTime = clock.seconds();
        currentSegmentIndex = 0;
        lastSegmentIndex = -1;
    }

    public 
    DriveSignal update(Pose2d poseEstimate, Pose2d poseVelocity) {
        Pose2d targetPose = null;
        DriveSignal driveSignal = null;

        SequenceSegment currentSegment = null;

        if (currentTrajectorySequence != null) {
            if (currentSegmentIndex >= currentTrajectorySequence.size()) {
                for (TrajectoryMarker marker : remainingMarkers) {
                    marker.getCallback().onMarkerReached();
                }

                remainingMarkers.clear();

                currentTrajectorySequence = null;
            }

            if (currentTrajectorySequence == null)
                return new DriveSignal();

            double now = clock.seconds();
            boolean isNewTransition = currentSegmentIndex != lastSegmentIndex;

            currentSegment = currentTrajectorySequence.get(currentSegmentIndex);

            if (isNewTransition) {
                currentSegmentStartTime = now;
                lastSegmentIndex = currentSegmentIndex;

                for (TrajectoryMarker marker : remainingMarkers) {
                    marker.getCallback().onMarkerReached();
                }

                remainingMarkers.clear();

                remainingMarkers.addAll(currentSegment.getMarkers());
                Collections.sort(remainingMarkers, (t1, t2) -> Double.compare(t1.getTime(), t2.getTime()));
            }

            double deltaTime = now - currentSegmentStartTime;

            if (currentSegment instanceof TrajectorySegment) {
                Trajectory currentTrajectory = ((TrajectorySegment) currentSegment).getTrajectory();

                if (isNewTransition)
                    follower.followTrajectory(currentTrajectory);

                if (!follower.isFollowing()) {
                    currentSegmentIndex++;

                    driveSignal = new DriveSignal();
                } else {
                    driveSignal = follower.update(poseEstimate, poseVelocity);
                    lastPoseError = follower.getLastError();
                }

                targetPose = currentTrajectory.get(deltaTime);
            } else if (currentSegment instanceof TurnSegment) {
                MotionState targetState = ((TurnSegment) currentSegment).getMotionProfile().get(deltaTime);

                turnController.setTargetPosition(targetState.getX());

                double correction = turnController.update(poseEstimate.getHeading());

                double targetOmega = targetState.getV();
                double targetAlpha = targetState.getA();

                lastPoseError = new Pose2d(0, 0, turnController.getLastError());

                Pose2d startPose = currentSegment.getStartPose();
                targetPose = startPose.copy(startPose.getX(), startPose.getY(), targetState.getX());

                driveSignal = new DriveSignal(
                        new Pose2d(0, 0, targetOmega + correction),
                        new Pose2d(0, 0, targetAlpha)
                );

                if (deltaTime >= currentSegment.getDuration()) {
                    currentSegmentIndex++;
                    driveSignal = new DriveSignal();
                }
            } else if (currentSegment instanceof WaitSegment) {
                lastPoseError = new Pose2d();

                targetPose = currentSegment.getStartPose();
                driveSignal = new DriveSignal();

                if (deltaTime >= currentSegment.getDuration()) {
                    currentSegmentIndex++;
                }
            }

            while (remainingMarkers.size() > 0 && deltaTime > remainingMarkers.get(0).getTime()) {
                remainingMarkers.get(0).getCallback().onMarkerReached();
                remainingMarkers.remove(0);
            }
        }

        poseHistory.add(poseEstimate);

        if (POSE_HISTORY_LIMIT > -1 && poseHistory.size() > POSE_HISTORY_LIMIT) {
            poseHistory.removeFirst();
        }

        return driveSignal;
    }

    public Pose2d getLastPoseError() {
        return lastPoseError;
    }

    public boolean isBusy() {
        return currentTrajectorySequence != null;
    }
}
