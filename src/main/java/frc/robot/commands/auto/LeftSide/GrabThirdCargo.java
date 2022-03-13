package frc.robot.commands.auto.LeftSide;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import frc.robot.commands.drivetrain.RunProfileCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.PivotPosition;

public class GrabThirdCargo extends RunProfileCommand {
    private final IntakeSubsystem intake;
    private final IndexerSubsystem indexer;

    public GrabThirdCargo(DriveSubsystem driveSubsystem, IntakeSubsystem intake, IndexerSubsystem indexer, Pose2d startPose) {
        super(driveSubsystem, startPose);
        
        this.indexer = indexer;
        this.intake = intake;

        addRequirements(intake);

        //System.out.println("GrabThirdCargo");

        TrajectoryBuilder tb1 = drivetrain.getTrajectoryBuilder(false, startPose);
        tb1
            .splineTo(new Vector2d(115, -295), Math.toRadians(300));
        
        addTrajectory(tb1.build(), true, true);

        //System.out.println("End GrabThirdCargo");
    }

    @Override
    public void execute() {
        super.execute();
        indexer.index();
    }

    int count = 0;

    @Override
    public boolean isFinished() {
        boolean isFinished = super.isFinished();

        if(isFinished && count > 50) {
            return true;
        }
        else if (isFinished) {
            count ++;
            return false;
        }
        else {
            return false;
        }
    }


    @Override
    public void initialize() {
        super.initialize();

        intake.intake();;

        startProfile();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        //intake.pivotTo(PivotPosition.Up);
    }

}