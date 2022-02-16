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

    public static final class IOConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

    }

    public static final class ShooterConstants {
 //       public static final int kPopperCanId = 8;
        public static final int kPopperCanId = 69;
        public static final int kTopCanId = 20;
        public static final int kBottomCanId = 15;

        public static final double kSlot1P = 0.255;
        public static final double kSlot1I = 0;
        public static final double kSlot1D = 0;
        public static final double kSlot1F = 0.05148;
    }

    public static final class DriveConstants {
        public static final int kRightOneCanId = 3;//!
        public static final int kRightTwoCanId = 6;//!
        public static final int kRightThreeCanId = 4;//!
        public static final int kLeftOneCanId = 2;//!
        public static final int kLeftTwoCanId = 1;//!
        public static final int kLeftThreeCanId = 5;//!

        public static final double kSlot1P = 0;
        public static final double kSlot1I = 0;
        public static final double kSlot1D = 0;
        public static final double kSlot1F = 0;
    }
    public static final class intakeConstants {
        public static final int kBarCanId = 7;//?
        public static final int kPivotCanId = 9;//!
 

        public static final double kSlot1P = 0;
        public static final double kSlot1I = 0;
        public static final double kSlot1D = 0;
        public static final double kSlot1F = 0;
    }

    public static final class ClimberConstants {
        public static final int kRightTopPwmChannel = 0;
        public static final int kRightBottomPwmChannel = 1;
        public static final int kLeftTopPwmChannel = 2;
        public static final int kLeftBottomPwmChannel = 3;

        public static final int kOuterTapeMeasureCanId = 0;
        public static final int kOuterPivotCanId = 0;
        public static final int kInnerTapeMeasureCanId = 0;
        public static final int kInnerPivotCanId = 0;
        public static final int kWinchCanId = 0;

        public static final int kInnerCanId = -0;//these are only here becasue I couldn't be bothered to alter all of ClimbSubsystem -A
        public static final int kOuterCanId = -0;


        public static final double kSlot1P = 0;
        public static final double kSlot1I = 0;
        public static final double kSlot1D = 0;
        public static final double kSlot1F = 0;

    }
    
    public static final class IndexerConstants {
        public static final int kIndexerCanId = 30;//!
        public static final int kSingleCanId = 8;//!
 
        public static final int kSensor1DioPort = 0;
        public static final int kSensor2DioPort = 1;
        public static final int kSensor3DioPort = 2;


        public static final double kSlot1P = 0;
        public static final double kSlot1I = 0;
        public static final double kSlot1D = 0;
        public static final double kSlot1F = 0;
    }
}
