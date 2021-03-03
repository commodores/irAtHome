// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class VelocityShooter extends SubsystemBase {
  /** Creates a new VelocityShooter. */
  private final WPI_TalonFX leftShooterMotor;
  private final WPI_TalonFX rightShooterMotor;

  private final Servo leftServo;
  private final Servo rightServo;
 

  public VelocityShooter() {

    leftShooterMotor = new WPI_TalonFX(ShooterConstants.kLeftShooterPort);
    rightShooterMotor = new WPI_TalonFX(ShooterConstants.kRightShooterPort);

    leftServo = new Servo(ShooterConstants.kLeftServo);
    rightServo = new Servo(ShooterConstants.kRightServo);
    
    leftShooterMotor.configFactoryDefault();
    rightShooterMotor.configFactoryDefault();
   
    leftShooterMotor.setNeutralMode(NeutralMode.Coast);
    rightShooterMotor.setNeutralMode(NeutralMode.Coast);
    
    leftShooterMotor.setInverted(false);
    rightShooterMotor.setInverted(true);
    rightShooterMotor.follow(leftShooterMotor);

    leftShooterMotor.setSensorPhase(false);
    
    leftShooterMotor.configPeakOutputForward(0);
    leftShooterMotor.configPeakOutputReverse(-1);
    leftShooterMotor.config_kP(0, ShooterConstants.kShooterP);
    leftShooterMotor.config_kI(0, ShooterConstants.kShooterI);
    leftShooterMotor.config_kD(0, ShooterConstants.kShooterD);
    leftShooterMotor.config_kF(0, ShooterConstants.kShooterF);
    leftShooterMotor.configAllowableClosedloopError(0, ShooterConstants.kAllowableError);
    leftShooterMotor.configMaxIntegralAccumulator(0, ShooterConstants.kMaxIntegralAccumulator);
    leftShooterMotor.configClosedLoopPeriod(0, ShooterConstants.kPIDLoopRate);
  

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run poop
  }

  public void shoot(double speed){
    leftShooterMotor.set(ControlMode.PercentOutput, speed);
  }

  public void velocityShoot(double velocity){
    leftShooterMotor.set(ControlMode.Velocity, RPMtoFalconUnits(velocity));
  }

  public void displayEncoders(){
    SmartDashboard.putNumber("Shooter Encoder", leftShooterMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Shooter RPM", falconUnitsToRPM(leftShooterMotor.getSelectedSensorVelocity()));
  }

  public double getRPM() {
    return falconUnitsToRPM(leftShooterMotor.getSelectedSensorVelocity());
  }

  public double falconUnitsToRPM(double sensorUnits) {
    return (sensorUnits / 2048.0) * 600.0;
  }

  public double RPMtoFalconUnits(double RPM) {
    return (RPM / 600.0) * 2048.0;
  }

  public void FullRetract() {
    leftServo.setPosition(.29);
    rightServo.setPosition(.29);
  }

  public void HalfExtend(){
    leftServo.setPosition(.6);
    rightServo.setPosition(.6);
  }

  public void whiteLineExtend(){
    leftServo.setPosition(.55);
    rightServo.setPosition(.55);
  }

  public void FullExtend(){
    leftServo.setPosition(.7);
    rightServo.setPosition(.7);
  }
  
  

}
