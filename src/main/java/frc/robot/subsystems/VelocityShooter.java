/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

/*
Subsystem for controlling to robot's shooter
 */

public class VelocityShooter extends SubsystemBase {
    
    // PID loop constants
    private double kF = 0.0665;  // 0.054      //  Gree: 0.0475;
    private double kP = 0.09;      //  0.4       //  0.00047
    private double kI = 0.0;                    //  0.0000287
    private double kD = 0.0;

    private double kS = 0.155;
    private double kV = 0.111;
    private double kA = 0.02;

    public int kI_Zone = 100;
    public int kAllowableError = 50;

    private TalonFX[] outtakeMotors = {
            new TalonFX(ShooterConstants.kLeftShooterPort),
            new TalonFX(ShooterConstants.kRightShooterPort),
    };

    private final Servo leftServo;
    private final Servo rightServo;

    public double rpmOutput;
    public double rpmTolerance = 50.0;

    private double setpoint;


//    public PIDController flywheelController = new PIDController(kP, kI, kD);
//    public SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    public VelocityShooter() {

        leftServo = new Servo(ShooterConstants.kLeftServo);
        rightServo = new Servo(ShooterConstants.kRightServo);

        // Setup shooter motors (Falcons)
        for (TalonFX outtakeMotor : outtakeMotors) {
            outtakeMotor.configFactoryDefault();
            outtakeMotor.setNeutralMode(NeutralMode.Coast);
            outtakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0));
            outtakeMotor.configVoltageCompSaturation(10);
            outtakeMotor.enableVoltageCompensation(true);
        }
        outtakeMotors[0].setInverted(false);
        outtakeMotors[1].setInverted(true);
        outtakeMotors[1].follow(outtakeMotors[0], FollowerType.PercentOutput);

        outtakeMotors[0].config_kF(0, kF);
        outtakeMotors[0].config_kP(0, kP);
        outtakeMotors[0].config_kI(0, kI);
        outtakeMotors[0].config_IntegralZone(0, kI_Zone);
        outtakeMotors[0].config_kD(0, kD);
        outtakeMotors[0].configAllowableClosedloopError(0, kAllowableError);
        outtakeMotors[0].configClosedloopRamp(0.2);
        outtakeMotors[1].configClosedloopRamp(0);
        outtakeMotors[1].configOpenloopRamp(0);
    }

    public double getMotorInputCurrent(int motorIndex) {
        return outtakeMotors[motorIndex].getSupplyCurrent();
    }

    public void setPower(double output) {
        outtakeMotors[0].set(ControlMode.PercentOutput, output);
    }

    public void setRPM(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getSetpoint() {
        return setpoint;
    }

    private void updateRPMSetpoint() {
        if (setpoint >= 0)
            outtakeMotors[0].set(ControlMode.Velocity, RPMtoFalconUnits(setpoint));
        else
            setPower(0);
    }

    public void setTestRPM() {
        outtakeMotors[0].set(ControlMode.Velocity, RPMtoFalconUnits(rpmOutput));
    }

    public double getTestRPM() {
        return rpmOutput;
    }

    public double getRPMTolerance() {
        return rpmTolerance;
    }

    public boolean encoderAtSetpoint(int motorIndex) {
        return (Math.abs(outtakeMotors[motorIndex].getClosedLoopError()) < 100.0);
    }

    public double getRPM(int motorIndex) {
        return falconUnitsToRPM(outtakeMotors[motorIndex].getSelectedSensorVelocity());
    }

    public double falconUnitsToRPM(double sensorUnits) {
        return (sensorUnits / 2048.0) * 600.0;
    }

    public double RPMtoFalconUnits(double RPM) {
        return (RPM / 600.0) * 2048.0;
    }
    
    @Override
    public void periodic() {
        updateRPMSetpoint();
        SmartDashboard.putNumber("Shooter Encoder", outtakeMotors[0].getSelectedSensorVelocity());
        SmartDashboard.putNumber("Shooter RPM", falconUnitsToRPM(outtakeMotors[0].getSelectedSensorVelocity()));
    }

    public void UnderGoal() {
        leftServo.setPosition(.29);
        rightServo.setPosition(.29);
      }
    
      public void whiteLineExtend(){
        leftServo.setPosition(.55);
        rightServo.setPosition(.55);
      }
    
      public void LongShot(){
        leftServo.setPosition(.6);
        rightServo.setPosition(.6);
      }

}