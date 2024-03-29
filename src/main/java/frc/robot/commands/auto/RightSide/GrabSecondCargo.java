package frc.robot.commands.auto.RightSide;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import frc.robot.commands.drivetrain.RunProfileCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.PivotPosition;

public class GrabSecondCargo extends RunProfileCommand {
    private final IntakeSubsystem intake;
    private final IndexerSubsystem indexer;

    public GrabSecondCargo(DriveSubsystem driveSubsystem, IntakeSubsystem intake, IndexerSubsystem indexer, Pose2d startPose) {
        super(driveSubsystem, startPose);
        
        this.indexer = indexer;
        this.intake = intake;

        addRequirements(intake);

        //System.out.println("GrabSecondCargo");

        TrajectoryBuilder tb1 = drivetrain.getTrajectoryBuilder(false, startPose);
        tb1
            .splineTo(new Vector2d(92, -145), Math.toRadians(270));
        
        addTrajectory(tb1.build(), true, true);

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
        intake.intake();

        startProfile();
    }
}