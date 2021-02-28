// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class VelocityShooter extends SubsystemBase {
  /** Creates a new VelocityShooter. */
  private final WPI_TalonFX shooterFXLeft = new WPI_TalonFX(ShooterConstants.kLeftShooterPort);
  private final WPI_TalonFX shooterFXRight = new WPI_TalonFX(ShooterConstants.kRightShooterPort);

  private final TalonFXSensorCollection leftShooterSensor;
 

  public VelocityShooter() {

    shooterFXLeft.configFactoryDefault();
    shooterFXRight.configFactoryDefault();
    
    shooterFXLeft.setNeutralMode(NeutralMode.Coast);
    shooterFXRight.setNeutralMode(NeutralMode.Coast);

    shooterFXLeft.setInverted(true);
    shooterFXRight.setInverted(true);

    shooterFXLeft.configOpenloopRamp(ShooterConstants.kShooterVoltageRampRate);
    shooterFXRight.configOpenloopRamp(ShooterConstants.kShooterVoltageRampRate);

    shooterFXRight.follow(shooterFXLeft);

    leftShooterSensor = shooterFXLeft.getSensorCollection();
    
    shooterFXLeft.configPeakOutputForward(0);
    shooterFXLeft.configPeakOutputReverse(-1);
    shooterFXLeft.config_kP(0, ShooterConstants.kShooterP);
    shooterFXLeft.config_kI(0, ShooterConstants.kShooterI);
    shooterFXLeft.config_kD(0, ShooterConstants.kShooterD);
    shooterFXLeft.config_kF(0, ShooterConstants.kShooterF);
    shooterFXLeft.configAllowableClosedloopError(0, ShooterConstants.kAllowableError);
    shooterFXLeft.configMaxIntegralAccumulator(0, ShooterConstants.kMaxIntegralAccumulator);
    shooterFXLeft.configClosedLoopPeriod(0, ShooterConstants.kPIDLoopRate);
  

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run poop
  }

  public void shoot(double speed){
    shooterFXLeft.set(ControlMode.PercentOutput, speed);
  }

  public void velocityShoot(double velocity){
    shooterFXLeft.set(ControlMode.Velocity, RPMtoFalconUnits(velocity));
  }

  public void displayEncoders(){
    SmartDashboard.putNumber("Shooter Encoder", leftShooterSensor.getIntegratedSensorVelocity());
    SmartDashboard.putNumber("Shooter RPM", falconUnitsToRPM(leftShooterSensor.getIntegratedSensorVelocity()));
  }

  public double getRPM() {
    return falconUnitsToRPM(leftShooterSensor.getIntegratedSensorVelocity());
  }

  public double falconUnitsToRPM(double sensorUnits) {
    return (sensorUnits / 2048.0) * 600.0;
  }

  public double RPMtoFalconUnits(double RPM) {
    return (RPM / 600.0) * 2048.0;
  }
  
  

}
