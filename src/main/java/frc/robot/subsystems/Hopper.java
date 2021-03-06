// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.DriveConstants;

public class Hopper extends SubsystemBase {
  /** Creates a new Hopper. */

  private final WPI_TalonSRX leftHopper;
  private final WPI_TalonSRX rightHopper;
  private final WPI_TalonSRX feederMotor;
  private PigeonIMU pigeon;

  public Hopper() {

    leftHopper = new WPI_TalonSRX(HopperConstants.kHopperLeftPort);
    rightHopper = new WPI_TalonSRX(HopperConstants.kHopperRightPort);
    feederMotor = new WPI_TalonSRX(HopperConstants.kFeederPort);

    pigeon = new PigeonIMU(leftHopper);

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
  public void runHopper(double speed){
    rightHopper.set(ControlMode.PercentOutput, speed);
  }

  public void stopHopper(){
    rightHopper.set(ControlMode.PercentOutput, 0.0);
  }
 
  //Commands For Feeder
  public void runFeed(double speed){
    feederMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stopFeeder(){
    feederMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void resetPigeon(){
    pigeon.configFactoryDefault();
  }
  
  public void resetDirection() {
    pigeon.setFusedHeading(0);
  }

  public double getDirection() {
    return -1 * Math.IEEEremainder(pigeon.getFusedHeading(), 360);
  }

  public double getFHeading(){
    return pigeon.getFusedHeading();
  }

  /**
  * Returns the turn rate of the robot.
  *
  * @return The turn rate of the robot, in degrees per second
  */
 public double getTurnRate() {
  double [] xyz_dps = new double [3];
  // getRawGyro returns in degrees/second
  pigeon.getRawGyro(xyz_dps);
  return xyz_dps[2] * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
