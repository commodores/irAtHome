// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */

  private final WPI_TalonSRX feederMotor;

  public Feeder() {

    feederMotor = new WPI_TalonSRX(HopperConstants.kFeederPort);

    feederMotor.configFactoryDefault();

    feederMotor.setNeutralMode(NeutralMode.Brake);
    feederMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 0, 0));
    feederMotor.configVoltageCompSaturation(8);
    feederMotor.enableVoltageCompensation(true);

    feederMotor.set(ControlMode.PercentOutput, 0.0);

  }
  
  //Commands For Feeder
  public void runFeed(double speed){
    feederMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stopFeeder(){
    feederMotor.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
