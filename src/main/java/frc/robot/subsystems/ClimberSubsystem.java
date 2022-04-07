// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
//import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.GeneralConstants;
import frc.robot.subsystems.ClimberArmSubsystem.PivotPosition;

public class ClimberSubsystem extends SubsystemBase {
  public enum ArmPosition {
    InitialGrab,
    BothMiddle,
    Hold,
    InnerGrab,
    OuterLatch,
    OuterGrab,
    LatchHigh,
    OuterLetGo,
    High,
    Coast
  }

  public enum WinchPosition {
    // 10526 per inch
    Hold(-5000),
    //InnerOut(-20000), //
    InitialGrab(180000),
    PullUp(-25000),
    OuterOutTwo(5000),
    PullupHigh(170000);

    private int value;
    private static Map map = new HashMap<>();

    private WinchPosition(int value) {
      this.value = value;
    }

    static {
      for (WinchPosition winchPosition : WinchPosition.values()) {
        map.put(winchPosition.value, winchPosition);
      }
    }

    public static WinchPosition valueOf(int winchPosition) {
      return (WinchPosition) map.get(winchPosition);
    }

    public int getValue() {
      return value;
    }
  }

  private ClimberArmSubsystem outer = new ClimberArmSubsystem(ClimberConstants.kOuterPivotCanId,
      ClimberArmSubsystem.Side.Outer);
  private ClimberArmSubsystem inner = new ClimberArmSubsystem(ClimberConstants.kInnerPivotCanId,
      ClimberArmSubsystem.Side.Inner);

  private WPI_TalonFX winch = new WPI_TalonFX(ClimberConstants.kWinchCanId, GeneralConstants.kCanBusAltName);

  //private static final double kWinchInitialSensorPos = 20000;
  private static final double kWinchCruiseVelocity = 18000; // Measured max velocity 22000
  private static final double kWinchAccelerationVelocity = 30000;
  private static final double kWinchP = 0.06;
  private static final double kWinchI = 0.0001;
  private static final double kWinchIZone = 4000;

  private static final double kWinchForwardLimit = 244000; // 106000
  private static final double kWinchReverseLimit = -25000;

  private static boolean climbStarted = false;

  private WinchPosition currentWinchPosition = WinchPosition.Hold;

  /** Creates a new ExampleSubsystem. */
  public ClimberSubsystem() {
    winch.configFactoryDefault(10);

    winch.setNeutralMode(NeutralMode.Brake);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

    config.motionCruiseVelocity = kWinchCruiseVelocity;
    config.motionAcceleration = kWinchAccelerationVelocity;

    config.forwardSoftLimitEnable = true;
    config.forwardSoftLimitThreshold = kWinchForwardLimit;
    config.reverseSoftLimitEnable = true;
    config.reverseSoftLimitThreshold = kWinchReverseLimit;

    config.slot0.kP = kWinchP;
    config.slot0.kI = kWinchI;
    config.slot0.integralZone = kWinchIZone;
    

    winch.configAllSettings(config, 10);

//    winch.setSelectedSensorPosition(kWinchInitialSensorPos, 0, 10);
  }

  public static void startClimb() {
    climbStarted = true;
  }

  public static boolean hasClimbStarted() {
    return climbStarted;
  }

  public void stop() {
    stop(false);
  }

  public void stop(boolean force) {
    if (!climbStarted || force) {
      // inner.setTape(0);
      // outer.setTape(0);
    }
    winch.set(ControlMode.MotionMagic, winch.getSelectedSensorPosition());
    // inner.holdPivot();
    // outer.holdPivot();
  }

  public boolean shouldLetGo() {
    return winch.getSelectedSensorPosition() < (WinchPosition.InitialGrab.value / 2);
  }

  public boolean isWinchHalfWay() {
    return winch.getSelectedSensorPosition() > (WinchPosition.InitialGrab.value / 2);
  }

  public void pivot(ArmPosition position) {
    pivot(position, false);
  }

  public void pivot(ArmPosition position, boolean force) {
    if (!climbStarted || force) {
      switch (position) {
        case OuterLatch:
          inner.setPivot(PivotPosition.LetGo);
          outer.setPivotCoast();
        case BothMiddle:
          inner.setPivot(PivotPosition.Middle);
          outer.setPivot(PivotPosition.Middle);
          break;
        case OuterGrab:
          outer.setPivot(PivotPosition.ForwardGrab);
          inner.setPivotCoast();
          break;
        case Coast:
          outer.setPivotCoast();
          inner.setPivotCoast();
          break;
        case LatchHigh:
          outer.setPivot(PivotPosition.Middle);
          inner.setPivot(PivotPosition.ForwardLatch);
          break;
        case High:
          outer.setPivot(PivotPosition.LetGo);
          inner.setPivot(PivotPosition.Middle);
          break;
        case Hold:
          outer.setPivot(PivotPosition.Hold);
          inner.setPivot(PivotPosition.Hold);
          break;
        case InitialGrab:
          outer.setPivot(PivotPosition.Grab);
          inner.setPivot(PivotPosition.ForwardGrab);
          break;
        case OuterLetGo:
          outer.setPivot(PivotPosition.LetGo);
          inner.setPivotCoast();
      }
    }
  }

  public void winch(WinchPosition position) {
    winch(position, false);
  }

  public void winch(WinchPosition position, boolean addFF) {
    currentWinchPosition = position;
    if (addFF) {
      winch.set(ControlMode.MotionMagic, position.value, DemandType.ArbitraryFeedForward, .25);
    } else {
      winch.set(ControlMode.MotionMagic, position.value);
    }
  }

  public WinchPosition getWinchPos() {
    return currentWinchPosition;
  }

  public double getWinchEnc() {
    return winch.getSelectedSensorPosition();
  }

  public void winchRaw(double power) {
    winch.set(ControlMode.PercentOutput, power);
  }

  public boolean isWinchFinished() {
    return Math.abs(currentWinchPosition.value - winch.getSelectedSensorPosition()) < 100;
  }

  public boolean isFinished() {
    return Math.abs(currentWinchPosition.value - winch.getSelectedSensorPosition()) < 100;
  }

  public double getWinchError() {
    return Math.abs(currentWinchPosition.value - winch.getSelectedSensorPosition());
  }

  public void winchStop() {
    winch.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Climb:Winch", winch.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Climb:WinchError", getWinchError());

    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getOuterPivotError() {
    return outer.getPivotError();
  }

  public double getInnerPivotError() {
    return inner.getPivotError();
  }
}
