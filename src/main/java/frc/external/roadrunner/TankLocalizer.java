package frc.external.roadrunner;

import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.TankKinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.Angle;

import frc.robot.subsystems.DriveSubsystem;

public class TankLocalizer implements Localizer  {
    private DriveSubsystem drive;
    private Pose2d pose = new Pose2d();
    private List<Double> lastWheelPositions = Collections.<Double>emptyList();
    private double lastHeading = Double.NaN;

    public TankLocalizer(DriveSubsystem drive) {
        this.drive = drive;
    }

    @Override
    public Pose2d getPoseEstimate() {
        return pose;
    }
    @Override
    public Pose2d getPoseVelocity() {
        // TODO Auto-generated method stub
        return null;
    }
    @Override
    public void setPoseEstimate(Pose2d pose) {
        lastWheelPositions = Collections.<Double>emptyList();
        lastHeading = Double.NaN;
        drive.setHeading(pose.getHeading());

        this.pose = pose;
    }
    @Override
    public void update() {
        var wheelPositions = drive.getWheelPositions();
        var heading = drive.getHeading();
        //System.out.println(String.format("%1$f,%2$f,%3$f", wheelPositions.get(0), wheelPositions.get(1), heading));
        if(!lastWheelPositions.isEmpty()) {
            var wheelDeltas = IntStream.range(0, wheelPositions.size()).mapToObj(i -> wheelPositions.get(i) - lastWheelPositions.get(i)).collect(Collectors.toUnmodifiableList());
            var robotPoseDelta = TankKinematics.wheelToRobotVelocities(wheelDeltas, drive.getTrackWidth());
            var finalHeadingDelta = Angle.normDelta(heading - lastHeading);
            pose = Kinematics.relativeOdometryUpdate(pose, new Pose2d(robotPoseDelta.vec(), finalHeadingDelta));
        }

        lastWheelPositions = wheelPositions;
        lastHeading = heading;
    }
}

/*
class TankLocalizer @JvmOverloads constructor(
        private val drive: TankDrive,
        private val useExternalHeading: Boolean = true
    ) : Localizer {
        private var _poseEstimate = Pose2d()
        override var poseEstimate: Pose2d
            get() = _poseEstimate
            set(value) {
                lastWheelPositions = emptyList()
                lastExtHeading = Double.NaN
                if (useExternalHeading) drive.externalHeading = value.heading
                _poseEstimate = value
            }
        override var poseVelocity: Pose2d? = null
            private set
        private var lastWheelPositions = emptyList<Double>()
        private var lastExtHeading = Double.NaN

        override fun update() {
            val wheelPositions = drive.getWheelPositions()
            val extHeading = if (useExternalHeading) drive.externalHeading else Double.NaN
            if (lastWheelPositions.isNotEmpty()) {
                val wheelDeltas = wheelPositions
                        .zip(lastWheelPositions)
                        .map { it.first - it.second }
                val robotPoseDelta = TankKinematics.wheelToRobotVelocities(wheelDeltas, drive.trackWidth)
                val finalHeadingDelta = if (useExternalHeading) {
                    Angle.normDelta(extHeading - lastExtHeading)
                } else {
                    robotPoseDelta.heading
                }
                _poseEstimate = Kinematics.relativeOdometryUpdate(
                    _poseEstimate,
                    Pose2d(robotPoseDelta.vec(), finalHeadingDelta)
                )
            }

            val wheelVelocities = drive.getWheelVelocities()
            val extHeadingVel = drive.getExternalHeadingVelocity()
            if (wheelVelocities != null) {
                poseVelocity = TankKinematics.wheelToRobotVelocities(wheelVelocities, drive.trackWidth)
                if (useExternalHeading && extHeadingVel != null) {
                    poseVelocity = Pose2d(poseVelocity!!.vec(), extHeadingVel)
                }
            }

            lastWheelPositions = wheelPositions
            lastExtHeading = extHeading
        }
    }
*/