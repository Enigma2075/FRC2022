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
    
    public Shoot(ShooterSubsystem shooter, IndexerSubsystem indexer) {
        this.shooter = shooter;
        this.indexer = indexer;

        addRequirements(shooter);
        addRequirements(indexer);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        boolean shooting = false;
        
        double targetAngle = 155;

        if(Math.abs(targetAngle - shooter.getTurretAngle()) < 1) {
            shooter.shoot(true);
            //shooting = shooter.shoot(.43); // 114.02 Distance
            shooting = true;
        }
        else {
            shooter.turret(targetAngle);
        }
       
        if(shooting) {
          indexer.index(true);
        }
        else {
          indexer.index();
        }    
    }

    int count = 0;

    @Override
    public boolean isFinished() {
        if(!indexer.hasCargo()) {
            count++;
        }
        else {
            count = 0;
        }

        if(count > 10) {
            return true;
        }
        else {
            return false;
        }
    }

    @Override 
    public void end(boolean interupted) {
        shooter.stop();
    }
}