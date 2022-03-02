package frc.robot.commands.auto;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.drivetrain.RunProfileCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends CommandBase {
    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;
    private final double targetAngle;
    private final double speed;
    private final int ballCount;

    private double wait;
    private boolean turretAtPosition = false;

    public Shoot(ShooterSubsystem shooter, IndexerSubsystem indexer, double targetAngle, double speed, int ballCount, double wait) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.targetAngle = targetAngle;
        this.speed = speed;
        this.wait = wait;
        this.ballCount = ballCount;

        addRequirements(shooter);
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        shooter.turret(targetAngle);
    }

    boolean shooting = false;
    boolean startBall = false;
    int ballsShot = 0;
    
    @Override
    public void execute() {
        if(!startBall && indexer.getPosition3()) {
            startBall = true;
        }
        else if (startBall && !indexer.getPosition3()){
            ballsShot++;
            startBall = false;
        }

        System.out.println(String.format("ballsShot:%d,startBall:%b", ballsShot, startBall));

        shooting = false;
        if (Math.abs(targetAngle - shooter.getTurretAngle()) < 1 || turretAtPosition) {
            turretAtPosition = true;

            if (speed == 0) {
                // shooting = shooter.shoot();
            } else {
                // shooting = shooter.shoot(speed); // 114.02 Distance
            }

            shooting = shooter.shoot(true);
        }

        if (shooting) {
            indexer.index(true);
        } else {
            indexer.index();
        }
    }

    int count = 0;

    @Override
    public boolean isFinished() {
        if(ballsShot == ballCount && count == wait) {
            return true;
        }
        else if(ballsShot == ballCount){
            count++;
            return false;
        }
        else {
            return false;
        }
        //if (!indexer.hasCargo() && ) {
        //    count++;
        //} else {
        //    count = 0;
        //}

        //if (count > wait && shooting) {
        //    return true;
        //} else {
        //    return false;
        //}
    }

    @Override
    public void end(boolean interupted) {
        shooter.stop();
    }
}