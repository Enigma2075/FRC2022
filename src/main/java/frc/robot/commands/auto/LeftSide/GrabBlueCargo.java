package frc.robot.commands.auto.LeftSide;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import frc.robot.commands.drivetrain.RunProfileCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.PivotPosition;

public class GrabBlueCargo extends RunProfileCommand {
    private final IntakeSubsystem intake;
    private final IndexerSubsystem indexer;

    public GrabBlueCargo(DriveSubsystem driveSubsystem, IntakeSubsystem intake, IndexerSubsystem indexer, Pose2d startPose) {
        super(driveSubsystem, startPose);
        
        this.indexer = indexer;
        this.intake = intake;

        addRequirements(intake);

        //System.out.println("GrabSecondCargo");
        
        double targetHeading = 342.5;

        addPointTurn(Math.toRadians((targetHeading - 225) * 1), false, false);

        TrajectoryBuilder tb1 = drivetrain.getTrajectoryBuilder(false, getEndPose());
        tb1
            .forward(120);   
        //.splineTo(new Vector2d(50, -127), Math.toRadians(targetHeading - 300));
        
        addTrajectory(tb1.build(), false, true);

        //addPointTurn(Math.toRadians(-180), false, true);

        //TrajectoryBuilder tb2 = drivetrain.getTrajectoryBuilder(false, getEndPose());
        //tb2
        //    .splineTo(new Vector2d(-140, -80), Math.toRadians(targetHeading - 180));
        
        //addTrajectory(tb2.build(), false, true);

        //System.out.println("End GrabSecondCargo");
    }

    @Override
    public void execute() {
        super.execute();
        indexer.index();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        intake.helpIntake();
    }

    @Override
    public void initialize() {
        super.initialize();

        indexer.index();

        startProfile();
    }
}