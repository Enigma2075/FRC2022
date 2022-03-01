package frc.robot.commands.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RightFull extends SequentialCommandGroup{
    DriveSubsystem drive;
    ShooterSubsystem shooter;
    IndexerSubsystem indexer;
    IntakeSubsystem intake;
    GyroSubsystem gyro;

    public RightFull(GyroSubsystem gyro, DriveSubsystem drive, ShooterSubsystem shooter, IndexerSubsystem indexer, IntakeSubsystem intake) {
        this.drive = drive;
        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;
        this.gyro = gyro;

        Pose2d startPose = new Pose2d(89.976, -22.85, 0);

        gyro.setYaw(Math.toDegrees(startPose.getHeading()));
        drive.setPosition(startPose);

        drive.resetEncoders();

        var shoot = new Shoot(shooter, indexer);
        var grabCargo = new GrabCargo(drive, intake, startPose);
        var grabCargoBack = new GrabSecondCargo(drive, intake, grabCargo.getEndPose());
        
        addCommands(shoot);
        addCommands(grabCargo);
        addCommands(grabCargoBack);
        
        //addCommands(new AutoShoot());
 
        //Command intakeCenterBalls = new ParallelDeadlineGroup(new LeftCenter(), new AutoIndex(), new IntakePowerCells());
        
        //addCommands(intakeCenterBalls);
        
        //addCommands(new AutoShoot());
    }

    @Override
    public void initialize() {
        //System.out.println("LeftFull");

        //drivetrain.setGlobalPosition(134, 27.6);
        
        super.initialize();
    }
}