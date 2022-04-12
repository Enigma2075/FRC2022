package frc.robot.commands.auto.LeftSide;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.auto.Shoot;
import frc.robot.commands.shooter.TurretCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveMode;

public class LeftFull extends SequentialCommandGroup{
    DriveSubsystem drive;
    ShooterSubsystem shooter;
    IndexerSubsystem indexer;
    IntakeSubsystem intake;
    GyroSubsystem gyro;
    private final Pose2d startPose;

    public LeftFull(GyroSubsystem gyro, DriveSubsystem drive, ShooterSubsystem shooter, IndexerSubsystem indexer, IntakeSubsystem intake) {
        this.drive = drive;
        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;
        this.gyro = gyro;
        
        // TODO: Update to be our real position.
        startPose = new Pose2d(-41.25, -85.5, Math.toRadians(225));
  
        var grabCargo = new GrabCargo(drive, intake, indexer, startPose);
        var shoot1 = new Shoot(shooter, indexer, 190, 0, 0, 2, 10);
        var grabSecondCargo = new GrabBlueCargo(drive, intake, indexer, grabCargo.getEndPose());
        var shoot2 = new Shoot(shooter, indexer, 100, .34, 38, 2, 10);
        var grabThirdCargo = new GrabThirdCargo(drive, intake, indexer, grabSecondCargo.getEndPose());
        var driveToShoot = new DriveToShoot(drive, intake, indexer, grabThirdCargo.getEndPose());
        var shoot3 = new Shoot(shooter, indexer, 200, 0, 10, 2, 10);
        
        addCommands(grabCargo);
        addCommands(shoot1);
        addCommands(grabSecondCargo);
        addCommands(grabThirdCargo);
        addCommands(shoot2);
        //addCommands(driveToShoot);
        //addCommands(shoot3);
        
        //addCommands(new AutoShoot());
 
        //Command intakeCenterBalls = new ParallelDeadlineGroup(new LeftCenter(), new AutoIndex(), new IntakePowerCells());
        
        //addCommands(intakeCenterBalls);
        
        //addCommands(new AutoShoot());
    }

    @Override
    public void initialize() {
        drive.resetEncoders();

        gyro.setYaw(Math.toDegrees(startPose.getHeading()));
        drive.setPosition(startPose);

        super.initialize();
    }

    @Override
    public void end(boolean interupted) {
        super.end(interupted);

        drive.setDriveMode(DriveMode.Normal);
        drive.setNeutralMode(NeutralMode.Coast);    
    }
}