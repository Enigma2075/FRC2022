package frc.robot.commands.auto.RightSide;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake.IntakeForceCommand;
import frc.robot.commands.auto.Shoot;
import frc.robot.commands.shooter.MoveTurretCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GyroSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveMode;

public class RightFull extends SequentialCommandGroup{
    DriveSubsystem drive;
    ShooterSubsystem shooter;
    IndexerSubsystem indexer;
    IntakeSubsystem intake;
    GyroSubsystem gyro;
    private final Pose2d startPose;

    public RightFull(GyroSubsystem gyro, DriveSubsystem drive, ShooterSubsystem shooter, IndexerSubsystem indexer, IntakeSubsystem intake) {
        this.drive = drive;
        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;
        this.gyro = gyro;

        startPose = new Pose2d(89.976, -22.85, Math.toRadians(0));

        var grabCargo = new GrabCargo(drive, intake, indexer, shooter, startPose);
        var shoot1 = new Shoot(shooter, indexer, 320, .478, 10, 2, 10);
        var grabSecondCargo = new GrabSecondCargo(drive, intake, indexer, grabCargo.getEndPose());
        var shoot2 = new Shoot(shooter, indexer, 220, 0, 0, 1, 15);
        var grabThirdCargo = new GrabThirdCargo(drive, intake, indexer, grabSecondCargo.getEndPose());
        var driveToShoot = new DriveToShoot(drive, intake, indexer, grabThirdCargo.getEndPose());
        var shoot3 = new Shoot(shooter, indexer, 210, 0, 0, 2, 10);
        
        addCommands(grabCargo);
        addCommands(new ParallelRaceGroup(shoot1, new IntakeForceCommand(intake)));
        addCommands(new ParallelCommandGroup(grabSecondCargo, new MoveTurretCommand(shooter, 220)));
        addCommands(shoot2);
        addCommands(new ParallelCommandGroup(grabThirdCargo, new MoveTurretCommand(shooter, 220)));
        addCommands(driveToShoot);
        addCommands(shoot3);
        
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