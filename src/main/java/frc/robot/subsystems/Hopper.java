// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

public class Hopper extends SubsystemBase {
  /** Creates a new Hopper. */

  private final WPI_TalonSRX leftHopper;
  private final WPI_TalonSRX rightHopper;
  private final WPI_TalonSRX feederMotor;

  public Hopper() {

    leftHopper = new WPI_TalonSRX(HopperConstants.kHopperLeftPort);
    rightHopper = new WPI_TalonSRX(HopperConstants.kHopperRightPort);
    feederMotor = new WPI_TalonSRX(HopperConstants.kFeederPort);

    leftHopper.configFactoryDefault();
    rightHopper.configFactoryDefault();
    feederMotor.configFactoryDefault();

    leftHopper.setNeutralMode(NeutralMode.Coast);
    rightHopper.setNeutralMode(NeutralMode.Coast);
    feederMotor.setNeutralMode(NeutralMode.Coast);

    rightHopper.setInverted(false);

    leftHopper.follow(rightHopper);

    rightHopper.set(ControlMode.PercentOutput, 0.0);

    feederMotor.set(ControlMode.PercentOutput, 0.0);

  }

  //Commands For Hopper
  public void hopperIn(){
    rightHopper.set(ControlMode.PercentOutput, .2);
  }

  public void hopperOut(){
    rightHopper.set(ControlMode.PercentOutput, -.2);
  }

  public void stopHopper(){
    rightHopper.set(ControlMode.PercentOutput, 0.0);
  }
 
  //Commands For Feeder
  public void feedIn(){
    feederMotor.set(ControlMode.PercentOutput, .2);
  }

  public void feedOut(){
    feederMotor.set(ControlMode.PercentOutput, -.2);
  }

  public void stopFeeder(){
    feederMotor.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
