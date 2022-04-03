package frc.robot.commands.auto.RightSide;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import frc.robot.commands.drivetrain.RunProfileCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem.PivotPosition;

public class GrabCargo extends RunProfileCommand {
    private final IntakeSubsystem intake;
    private final IndexerSubsystem indexer;
    private final ShooterSubsystem shooter;

    public GrabCargo(DriveSubsystem driveSubsystem, IntakeSubsystem intake, IndexerSubsystem indexer, ShooterSubsystem shooter, Pose2d startPose) {
        super(driveSubsystem, startPose);
        
        this.indexer = indexer;
        this.intake = intake;
        this.shooter = shooter;

        addRequirements(intake);

        //System.out.println("GrabCargo");

        TrajectoryBuilder tb1 = drivetrain.getTrajectoryBuilder(false, startPose);
        tb1
                .forward(46);
        addTrajectory(tb1.build(), false);

        addPointTurn(Math.toRadians(-140), true, true);

        //TrajectoryBuilder tb2 = drivetrain.getTrajectoryBuilder(false, getEndPose());
        //tb2
        //        .forward(10);
        //addTrajectory(tb2.build(), true);

        //System.out.println("End GrabCargo");
    }

    @Override
    public void execute() {
        super.execute();
        indexer.index();

        var pose = drivetrain.getCurrentGlobalPosition();
        var headingDeg = Math.toDegrees(pose.getHeading());
        if(headingDeg < 355 && headingDeg > 340) {
            intake.helpIntake();
        }
    }

    @Override
    public void initialize() {
        super.initialize();

        shooter.turret(320);

        intake.intake();

        startProfile();
    }

    @Override
    public void end(boolean interrupted) {
        intake.intake();

        super.end(interrupted);
    }
}