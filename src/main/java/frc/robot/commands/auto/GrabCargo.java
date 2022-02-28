package frc.robot.commands.auto;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import frc.robot.commands.drivetrain.RunProfileCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class GrabCargo extends RunProfileCommand {
    private final IntakeSubsystem intake;

    public GrabCargo(DriveSubsystem driveSubsystem, IntakeSubsystem intake) {
        super(driveSubsystem);
        
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.intake();

        super.initialize();
        clearTrajectories();

        System.out.println("GrabCargo");

        TrajectoryBuilder tb1 = drivetrain.getTrajectoryBuilder(false);
        tb1
                // //289 inches to the balls -24
                .forward(42);
        //        .splineTo(new Vector2d(0, 48), 90);
                //.lineTo(new Vector2d(24, 0));
        // //.setReversed(true)
        // //.splineTo(new Pose2d(134,228.5, Math.toRadians(-90));

        addTrajectory(tb1.build());

        // TrajectoryBuilder tb2 = drivetrain.getTrajectoryBuilder(true, 240 - 24, 27.6,
        // 180);
        // tb2
        // //.lineTo(new Vector2d(134, 27.6));
        // .splineTo(new Pose2d(174,228, Math.toRadians(90)));
        // //.lineTo(new Vector2d(174, 228));

        // addTrajectory(tb2.build());

        startProfile();
    }
}