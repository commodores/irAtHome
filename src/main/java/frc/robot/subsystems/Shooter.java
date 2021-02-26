// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final WPI_TalonFX leftShooterMotor;
  private final WPI_TalonFX rightShooterMotor;

  private final Servo leftServo;
  private final Servo rightServo;
  
  public Shooter() {

    leftShooterMotor = new WPI_TalonFX(ShooterConstants.kLeftShooterPort);
    rightShooterMotor = new WPI_TalonFX(ShooterConstants.kRightShooterPort);

    leftServo = new Servo(ShooterConstants.kLeftServo);
    rightServo = new Servo(ShooterConstants.kRightServo);
    
    leftShooterMotor.configFactoryDefault();
    rightShooterMotor.configFactoryDefault();
   
    leftShooterMotor.setNeutralMode(NeutralMode.Coast);
    rightShooterMotor.setNeutralMode(NeutralMode.Coast);
    
    rightShooterMotor.setInverted(true);
    rightShooterMotor.follow(leftShooterMotor);
    
    leftShooterMotor.configOpenloopRamp(ShooterConstants.SHOOTER_VOLTAGE_RAMP_RATE);
    rightShooterMotor.configOpenloopRamp(ShooterConstants.SHOOTER_VOLTAGE_RAMP_RATE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //leftShooterMotor.set(.1);
  }

  public void set(double speed) {
    leftShooterMotor.set(speed);
  }
  
  public void stop() {
    leftShooterMotor.set(0);
  }

  public void NiceShot() {
    leftServo.setPosition(1);
    rightServo.setPosition(1);
  }

  public void WhataShot(){
    leftServo.setPosition(.75);
    rightServo.setPosition(.75);
  }

  public void WhatShot(){
    leftServo.setPosition(.25);
    rightServo.setPosition(.25);
  }

  public double getAverageSpeed() {
    return (getLeftSpeed() + getRightSpeed()) / 2.0;
  }

  public double getLeftSpeed() {
    return leftShooterMotor.getSensorCollection().getIntegratedSensorVelocity();
  }

  public double getRightSpeed() {
    return rightShooterMotor.getSensorCollection().getIntegratedSensorVelocity();
  }

  

}
