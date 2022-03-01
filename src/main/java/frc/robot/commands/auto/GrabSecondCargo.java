package frc.robot.commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import frc.robot.commands.drivetrain.RunProfileCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class GrabSecondCargo extends RunProfileCommand {
    private final IntakeSubsystem intake;

    public GrabSecondCargo(DriveSubsystem driveSubsystem, IntakeSubsystem intake, Pose2d startPose) {
        super(driveSubsystem, startPose);
        
        this.intake = intake;

        addRequirements(intake);

        System.out.println("GrabSecondCargo");

        addPointTurn(-120, true, false);

        TrajectoryBuilder tb1 = drivetrain.getTrajectoryBuilder(false, getEndPose());
        tb1
                // //289 inches to the balls -24
                
                .splineTo(new Vector2d(88, -90), Math.toRadians(270));
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

        //addTrajectory(tb1.build(), true, true);

        System.out.println("End GrabSecondCargo");
    }

    @Override
    public void initialize() {
        super.initialize();

        intake.intake();

        startProfile();
    }
}