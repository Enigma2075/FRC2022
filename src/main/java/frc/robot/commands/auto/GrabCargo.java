package frc.robot.commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import frc.robot.commands.drivetrain.RunProfileCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class GrabCargo extends RunProfileCommand {
    private final IntakeSubsystem intake;
    private final IndexerSubsystem indexer;

    public GrabCargo(DriveSubsystem driveSubsystem, IntakeSubsystem intake, IndexerSubsystem indexer, Pose2d startPose) {
        super(driveSubsystem, startPose);
        
        this.indexer = indexer;
        this.intake = intake;

        addRequirements(intake);

        System.out.println("GrabCargo");

        TrajectoryBuilder tb1 = drivetrain.getTrajectoryBuilder(false, startPose);
        tb1
                // //289 inches to the balls -24
                .forward(46);
        //        .splineTo(new Vector2d(0, 48), 90);
                //.lineTo(new Vector2d(24, 0));
        // //.setReversed(true)
        // //.splineTo(new Pose2d(134,228.5, Math.toRadians(-90));



        // TrajectoryBuilder tb2 = drivetrain.getTrajectoryBuilder(true, 240 - 24, 27.6,
        // 180);
        // tb2
        // //.lineTo(new Vector2d(134, 27.6));
        // .splineTo(new Pose2d(174,228, Math.toRadians(90)));
        // //.lineTo(new Vector2d(174, 228));

        // addTrajectory(tb2.build());

        addTrajectory(tb1.build(), true);
        System.out.println("End GrabCargo");
    }

    @Override
    public void execute() {
        super.execute();
        indexer.index();
    }

    @Override
    public void initialize() {
        super.initialize();

        intake.intake();

        startProfile();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}