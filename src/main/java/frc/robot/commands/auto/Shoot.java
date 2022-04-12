package frc.robot.commands.auto;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.drivetrain.RunProfileCommand;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Shoot extends CommandBase {
    private final ShooterSubsystem shooter;
    private final IndexerSubsystem indexer;
    private final double targetAngle;
    private final double speed;
    private final double hoodPosition;
    private final int ballCount;

    private double wait;
    private boolean turretAtPosition = false;

    private boolean startedShooting = false;

    public Shoot(ShooterSubsystem shooter, IndexerSubsystem indexer, double targetAngle, double speed, double hoodPosition, int ballCount, double wait) {
        this.shooter = shooter;
        this.indexer = indexer;
        this.targetAngle = targetAngle;
        this.speed = speed;
        this.hoodPosition = hoodPosition;
        this.wait = wait;
        this.ballCount = ballCount;

        addRequirements(shooter);
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
        shooter.setVision(true);
        shooter.turret(targetAngle);
    }

    boolean shooting = false;
    boolean startBall = false;
    double noBalls = 0;
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
        
        //System.out.println(String.format("ballsShot:%d,startBall:%b", ballsShot, startBall));

        shooting = false;
        if ((Math.abs(targetAngle - shooter.getTurretAngle()) < 5 || turretAtPosition) && !startedShooting) {
            turretAtPosition = true;

            if (speed == 0) {
                shooter.updateVisionData();
                shooting = shooter.shoot();
            } else {
                shooting = shooter.shoot(speed, hoodPosition); // 114.02 Distance
            }

            //safe shot when we don't relly have a target.
            //shooting = shooter.shoot(true);
        }

        if (shooting || startedShooting) {
            startedShooting = true;
            indexer.index(true);
        } else {
            indexer.index();
        }

        if(!indexer.hasCargo() && noBalls == 0) {
            noBalls = Timer.getFPGATimestamp();
        }
        else if(indexer.hasCargo()) {
            noBalls = 0;
        }
    }

    int count = 0;

    @Override
    public boolean isFinished() {
        if(ballsShot >= ballCount && count >= wait) {
            return true;
        }
        else if(ballsShot >= ballCount){
            count++;
            return false;
        }

        if(noBalls != 0 && Timer.getFPGATimestamp() - noBalls > 1.5) {
            return true;
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
        shooter.setVision(false);
        shooter.stop();
    }
}