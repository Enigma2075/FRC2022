// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class GeneralConstants {
        public static final String kCanBusAltName = "canivore";
        public static final String kCanBusRioName = "rio";
    }

    public static final class IOConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static final class GyroConstants {
        public static final int kGyroCanId = 1;
    }

    public static final class ShooterConstants {
        public static final int kTurretCanId = 7;
        public static final int kHoodCanId = 1;
        public static final int kPopperCanId = 2;
        public static final int kTopCanId = 10;
        public static final int kBottomCanId = 9;
        public static final int kLimeLightRightPwmPort = 1;
        public static final int kLimeLightLeftPwmPort = 0;

        public static final double kSlot1P = 0.255;
        public static final double kSlot1I = 0;
        public static final double kSlot1D = 0;
        public static final double kSlot1F = 0.05148;
    }

    public static final class DriveConstants {
        public static final int kRightOneCanId = 1;
        public static final int kRightTwoCanId = 2;
        public static final int kRightThreeCanId = 3;
        public static final int kLeftOneCanId = 4;
        public static final int kLeftTwoCanId = 5;
        public static final int kLeftThreeCanId = 6;

        public static final double kSlot1P = 0;
        public static final double kSlot1I = 0;
        public static final double kSlot1D = 0;
        public static final double kSlot1F = 0;
        
        public static final double kProfileResolution = 10.0; // Time in milliseconds for each point
    }

    public static final class intakeConstants {
        public static final int kBarCanId = 1;
        public static final int kPivotCanId = 16;
 

        public static final double kSlot1P = 0;
        public static final double kSlot1I = 0;
        public static final double kSlot1D = 0;
        public static final double kSlot1F = 0;
    }

    public static final class ClimberConstants {
        public static final int kOuterTapeCanId = 14;
        public static final int kOuterPivotCanId = 12;
        public static final int kInnerTapeCanId = 13;
        public static final int kInnerPivotCanId = 11;
        public static final int kWinchCanId = 8;

        public static final int kInnerCanId = -0;//these are only here becasue I couldn't be bothered to alter all of ClimbSubsystem -A
        public static final int kOuterCanId = -0;


        public static final double kSlot1P = 0;
        public static final double kSlot1I = 0;
        public static final double kSlot1D = 0;
        public static final double kSlot1F = 0;

    }
    
    public static final class IndexerConstants {
        public static final int kIndexerCanId = 15;
        public static final int kSingulizerCanId = 2;
 
        public static final int kSensor1DioPort = 0;
        public static final int kSensor2DioPort = 1;
        public static final int kSensor3DioPort = 2;


        public static final double kSlot1P = 0;
        public static final double kSlot1I = 0;
        public static final double kSlot1D = 0;
        public static final double kSlot1F = 0;
    }
}
