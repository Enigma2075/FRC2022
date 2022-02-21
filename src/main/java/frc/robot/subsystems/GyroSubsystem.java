// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GeneralConstants;
import frc.robot.Constants.GyroConstants;

public class GyroSubsystem extends SubsystemBase { 
  private final Pigeon2 pigeon = new Pigeon2(GyroConstants.kGyroCanId, GeneralConstants.kCanBusAltName);

  public GyroSubsystem() {
    
  }

public void setYaw() {
  pigeon.setYaw(0);
}

public double getYaw() {
  return pigeon.getYaw();
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    debug();
  }

  public void debug() {
    SmartDashboard.putNumber("Gyro:Yaw", pigeon.getYaw());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
