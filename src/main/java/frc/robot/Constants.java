// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.commands.*;
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

    }

    public static final class ShooterConstants {
        public static final int kRightShooterPort = 5;
        public static final int kLeftShooterPort = 6;
        //public static final int kTopShooterPort = 7;

        public static final int kLeftServo = 1;
        public static final int kRightServo = 2;


        public static final double shooterP = 0.0011;
        public static final double shooterI = 0;
        public static final double shooterD = 4;
        public static final double shooterF = 0.00017;
        public static final double MAX_RPM = 6200;
        public static final double SHOOTER_VOLTAGE_RAMP_RATE = .2;
    }

    public static final class OIConstants{
        public static final int kDriverControllerPort = 0;
        public static final int kLeftJoystickPort = 1;
        public static final int kRightJoystickPort = 2;
    }
    
    public static final class HopperConstants{
        public static final int kHopperLeftPort = 7;
        public static final int kHopperRightPort = 8;
        public static final int kFeederPort = 9;
    }

    public static final class IntakeConstants{
        public static final int kIntakePort = 5;
        public static final int kIntakeSolenoidPort = 1;
    }
    
    public static final class CompressorConstant{
        public static final int kCompressor = 0;
    }

    public static final class GainsConstants{
        
    }


}
