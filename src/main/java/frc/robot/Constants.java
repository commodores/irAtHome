// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants {
        public static final int kLeftMasterPort = 3;
        public static final int kLeftSlavePort = 4;

        public static final int kRightMasterPort = 2;
        public static final int kRightSlavePort = 1;

        public static final int kPigeonPort = 12;

        public static final int driveTimeout = 30;

        public static final int kEncoderCPR = 2048; //https://docs.ctre-phoenix.com/en/latest/ch14_MCSensor.html
        public static final double kWheelDiameterMeters = 0.1016; //4 inches
        public static final double kGearReduction = 7;
        public static final double kEncoderDistancePerPulse = ((kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR) / kGearReduction; 
        public static final double kWheelDistancePerPulse = kEncoderDistancePerPulse/ kGearReduction; //DISTANCE PER PULSE OF WHEEL= (OUTER CIRCUMFERENCE OF WHEEL)/(ENCODER CPR*GEAR REDUCTION)

        public static final double ksVolts = 0.699;
        public static final double kvVoltSecondsPerMeter = 2.36;
        public static final double kaVoltSecondsSquaredPerMeter = 0.22;
        public static final double kPDriveVel = .5; //2.24;   // 2.29

        public static final double kTrackwidthMeters = 0.5541772;//CAD //.5715 Tape //0.59825 From Char Tool
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);

        public static final double kDriveTrainGain = .015;

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double kMaxSpeedMetersPerSecond = 1.75; //3 Slalom
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.75; //2 Slalom 

        public static final boolean kGyroReversed = true;

        public static StatorCurrentLimitConfiguration TALON_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 36.5, 36.5, 0.25);
        

    }

    public static final class ShooterConstants {
        public static final int kRightShooterPort = 5;
        public static final int kLeftShooterPort = 6;

        public static final int kHoodSolendoidPort = 0;

        public static final int kShooterSolenoidPort = 0;

        public static final double kShooterVoltageRampRate = 0.2;


        public static final double kShooterP = 0.0075;
        public static final double kShooterI = 0; //1
        public static final double kShooterD = 0; //5
        public static final double kShooterF = 0.0585; // 0.0513

        public static final int kAllowableError = 50;
        public static final int kPIDLoopRate = 10; //In ms
        public static final int kMaxIntegralAccumulator = 1000;

        public static final double klimeLightHeight = 18.5;//inches
        public static final double ktargetHeight = 89.75;//inches
        public static final double kCameraAngle = 25;
    }
    public static final class ClimberConstants{
        public static final int kClimberPort = 14;
    }

    public static final class OIConstants{
        public static final int kDriverControllerPort = 0;
       public static final int kDriverController2Port = 1;
    }
    
    public static final class HopperConstants{
        public static final int kHopperLeftPort = 7;
        public static final int kHopperRightPort = 9;
        public static final int kFeederPort = 8;
    }

    public static final class IntakeConstants{
        public static final int kLeftIntakePort = 11;
        public static final int kRightIntakePort =10;
        public static final int kIntakeSolenoidPort = 1;
    }
    
    public static final class CompressorConstant{
        public static final int kCompressor = 0;
    }

    public static final class GainsConstants{
        
    }


}
