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

    public static final class DriveConstants {
        public static final int kLeftMasterPort = 3;
        public static final int kLeftSlave0Port = 4;

        public static final int kRightMasterPort = 2;
        public static final int kRightSlave0Port = 1;

        public static final int kPigeonPort = 7;

        public static final int kEncoderCPR = 2048; //https://docs.ctre-phoenix.com/en/latest/ch14_MCSensor.html
        public static final double kWheelDiameterMeters = 0.10; //6 inches
        public static final double kGearReduction = 7;
        public static final double kEncoderDistancePerPulse = ((kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR) / kGearReduction; 
        public static final double kWheelDistancePerPulse = kEncoderDistancePerPulse/ kGearReduction; //DISTANCE PER PULSE OF WHEEL= (OUTER CIRCUMFERENCE OF WHEEL)/(ENCODER CPR*GEAR REDUCTION)
        public static final double kDriveTrainGain = .015;

    }

    public static final class ShooterConstants {
        public static final int kRightShooterPort = 5;
        public static final int kLeftShooterPort = 6;
        //public static final int kTopShooterPort = 7;

        public static final int kLeftServo = 0;
        public static final int kRightServo = 1;


        public static final double kShooterP = 0.0001;
        public static final double kShooterI = 0.0;
        public static final double kShooterD = 0.0;
        public static final double kShooterF = 0.00017;
        public static final double kShooterVoltageRampRate = .2;

        public static final int kAllowableError = 150;
        public static final int kPIDLoopRate = 10; //In ms
        public static final int kMaxIntegralAccumulator = 1000;
    }

    public static final class OIConstants{
        public static final int kDriverControllerPort = 0;
        public static final int kLeftJoystickPort = 1;
        public static final int kRightJoystickPort = 2;
    }
    
    public static final class HopperConstants{
        public static final int kHopperLeftPort = 7;
        public static final int kHopperRightPort = 9;
        public static final int kFeederPort = 8;
    }

    public static final class IntakeConstants{
        public static final int kLeftIntakePort = 11;
        public static final int kRightIntakePort =10;
        public static final int kIntakeSolenoidPort = 0;
    }
    
    public static final class CompressorConstant{
        public static final int kCompressor = 0;
    }

    public static final class GainsConstants{
        
    }


}
