package frc.robot.commands.auto.LeftSide;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import frc.robot.commands.drivetrain.RunProfileCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.PivotPosition;

public class DriveToShoot extends RunProfileCommand {
    private final IntakeSubsystem intake;
    private final IndexerSubsystem indexer;

    public DriveToShoot(DriveSubsystem driveSubsystem, IntakeSubsystem intake, IndexerSubsystem indexer, Pose2d startPose) {
        super(driveSubsystem, startPose);
        
        this.indexer = indexer;
        this.intake = intake;

        addRequirements(intake);

        //System.out.println("DriveToShoot");

        TrajectoryBuilder tb1 = drivetrain.getTrajectoryBuilder(true, startPose);
        tb1
            .splineTo(new Vector2d(88, -90), Math.toRadians(90));
        
        addTrajectory(tb1.build(), true, true);

        //System.out.println("End DriveToShoot");
    }

    @Override
    public void execute() {
        super.execute();
        indexer.index();
    }

    @Override
    public void initialize() {
        super.initialize();

        intake.pivotTo(PivotPosition.Up);

        startProfile();
    }
}