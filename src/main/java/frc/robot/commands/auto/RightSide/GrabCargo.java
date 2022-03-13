package frc.robot.commands.auto.RightSide;

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