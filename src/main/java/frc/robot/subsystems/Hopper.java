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

public class Hopper extends SubsystemBase {
  /** Creates a new Hopper. */

  private final WPI_TalonSRX leftHopper;
  private final WPI_TalonSRX rightHopper;
  

  public Hopper() {

    leftHopper = new WPI_TalonSRX(HopperConstants.kHopperLeftPort);
    rightHopper = new WPI_TalonSRX(HopperConstants.kHopperRightPort);
    

    leftHopper.configFactoryDefault();
    rightHopper.configFactoryDefault();
    

    leftHopper.setNeutralMode(NeutralMode.Brake);
    leftHopper.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 0, 0));
    leftHopper.configVoltageCompSaturation(8);
    leftHopper.enableVoltageCompensation(true);

    rightHopper.setNeutralMode(NeutralMode.Brake);
    rightHopper.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 0, 0));
    rightHopper.configVoltageCompSaturation(8);
    rightHopper.enableVoltageCompensation(true);
    

    rightHopper.setInverted(false);

    leftHopper.set(ControlMode.PercentOutput, 0.0);
    rightHopper.set(ControlMode.PercentOutput, 0.0);

    

  }

  //Commands For Hopper
  public void runHopper(double speed){
    rightHopper.set(ControlMode.PercentOutput, speed);
    leftHopper.set(ControlMode.PercentOutput, speed );
  }

  public void stopHopper(){
    rightHopper.set(ControlMode.PercentOutput, 0.0);
    leftHopper.set(ControlMode.PercentOutput, 0.0);
  }
 
  public void runAutoHopper(double speed){
    rightHopper.set(ControlMode.PercentOutput, -speed);
    leftHopper.set(ControlMode.PercentOutput, speed * .5);

  }
  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
