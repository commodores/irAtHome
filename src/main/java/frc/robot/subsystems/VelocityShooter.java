// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFXPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.*;
import frc.robot.Constants.ShooterConstants;

public class VelocityShooter extends SubsystemBase {
  /** Creates a new VelocityShooter. */
  private final WPI_TalonFX shooterFXLeft = new WPI_TalonFX(ShooterConstants.kLeftShooterPort);
  private final WPI_TalonFX shooterFXRight = new WPI_TalonFX(ShooterConstants.kRightShooterPort);

  private final TalonFXSensorCollection leftShooterSensor;
  private final TalonFXSensorCollection rightShooterSensor;

  //private CANCoder m_Coder;

  //hard set PID values
  double pValue = 0.06;
  double iValue = 0.02;
  double dValue = 0.8;
  double fValue = 0.05;
  int allowableError = 150;
  int PIDLoopRate = 10; //In ms
  int maxIntegralAccumulator = 1000;
 

  public VelocityShooter() {

    shooterFXLeft.configFactoryDefault();
    shooterFXRight.configFactoryDefault();
    
    shooterFXLeft.setNeutralMode(NeutralMode.Coast);
    shooterFXRight.setNeutralMode(NeutralMode.Coast);

    shooterFXLeft.setInverted(true);
    shooterFXRight.setInverted(true);

    shooterFXRight.follow(shooterFXLeft);


    rightShooterSensor = shooterFXRight.getSensorCollection();
    shooterFXRight.configPeakOutputForward(1);
    shooterFXRight.configPeakOutputReverse(0);
    shooterFXRight.config_kP(0, pValue);
    shooterFXRight.config_kI(0, iValue);
    shooterFXRight.config_kD(0, dValue);
    shooterFXRight.config_kF(0, fValue);
    shooterFXRight.configAllowableClosedloopError(0, 0);
    shooterFXRight.configMaxIntegralAccumulator(0, maxIntegralAccumulator);
    shooterFXRight.configClosedLoopPeriod(0, PIDLoopRate);


    leftShooterSensor = shooterFXLeft.getSensorCollection();
    shooterFXLeft.configPeakOutputForward(0);
    shooterFXLeft.configPeakOutputReverse(-1);
    shooterFXLeft.config_kP(0, pValue);
    shooterFXLeft.config_kI(0, iValue);
    shooterFXLeft.config_kD(0, dValue);
    shooterFXLeft.config_kF(0, fValue);
    shooterFXLeft.configAllowableClosedloopError(0, 0);
    shooterFXLeft.configMaxIntegralAccumulator(0, maxIntegralAccumulator);
    shooterFXLeft.configClosedLoopPeriod(0, PIDLoopRate);
  

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run poop
  }

  public void shoot(double rightSpeed, double leftSpeed){
    shooterFXLeft.set(ControlMode.PercentOutput, leftSpeed);
    shooterFXRight.set(ControlMode.PercentOutput, rightSpeed);
  }

  public void displayEncoders(){
    SmartDashboard.putNumber("Right Shooter Encoder", rightShooterSensor.getIntegratedSensorVelocity());
    SmartDashboard.putNumber("Left Shooter Encoder", leftShooterSensor.getIntegratedSensorVelocity());
    
  }
  
  

}
