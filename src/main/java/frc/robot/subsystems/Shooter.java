// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Servo;
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
    
    leftShooterMotor.configOpenloopRamp(ShooterConstants.kShooterVoltageRampRate);
    rightShooterMotor.configOpenloopRamp(ShooterConstants.kShooterVoltageRampRate);
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

  public void UnderGoal() {
    leftServo.setPosition(.29);
    rightServo.setPosition(.29);
  }

  public void whiteLineExtend(){
    leftServo.setPosition(.55);
    rightServo.setPosition(.55);
  }

  public void LongShot(){
    leftServo.setPosition(.63);
    rightServo.setPosition(.63);
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
