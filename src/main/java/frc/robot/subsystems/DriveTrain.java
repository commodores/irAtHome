// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */

  private final WPI_TalonFX rightMasterMotor;
  private final WPI_TalonFX leftMasterMotor;

  private final WPI_TalonFX rightSlaveMotor;
  private final WPI_TalonFX leftSlaveMotor;

  private final DifferentialDrive m_drive;


  public DriveTrain() {

  //DriveTrain Electronics
    rightMasterMotor = new WPI_TalonFX(DriveConstants.kRightMasterPort);
    rightSlaveMotor = new WPI_TalonFX(DriveConstants.kRightSlave0Port);

    leftMasterMotor = new WPI_TalonFX(DriveConstants.kLeftMasterPort);
    leftSlaveMotor = new WPI_TalonFX(DriveConstants.kLeftSlave0Port);


  //Set Electronics To Default
    rightMasterMotor.configFactoryDefault();
    rightSlaveMotor.configFactoryDefault();

    leftMasterMotor.configFactoryDefault();
    leftSlaveMotor.configFactoryDefault();
  //Set Electronics To Open Ramp Rate
    leftMasterMotor.configOpenloopRamp(2.0);

    rightMasterMotor.configOpenloopRamp(2.0);

  //Set Electronics To Follow Mode
    rightSlaveMotor.follow(rightMasterMotor);

    leftSlaveMotor.follow(leftMasterMotor);
    
  //Set Electronics To Brake Mode
    rightMasterMotor.setNeutralMode(NeutralMode.Brake);

    leftMasterMotor.setNeutralMode(NeutralMode.Brake);
  
  //Set Electronics To Follow Control Mode
    rightMasterMotor.set(ControlMode.PercentOutput, 0.0);

    leftMasterMotor.set(ControlMode.PercentOutput, 0.0);
  
  //Setting Default Command
    setDefaultCommand(new DriveManual(this));
  
  // Finalizing DriveTrain
    leftMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    
    m_drive = new DifferentialDrive(leftMasterMotor, rightMasterMotor);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Left Encoder", getLeftDistance());
    SmartDashboard.putNumber("Right Encoder", getRightDistance());
    SmartDashboard.putNumber("Heading", RobotContainer.m_hopper.getDirection());
  }

  public void arcadeDrive(double moveSpeed, double rotateSpeed)
  {
    m_drive.arcadeDrive(moveSpeed,-rotateSpeed);
  }

  public void tankDrive(double leftSpeed, double rightSpeed)
  {
    m_drive.tankDrive(leftSpeed,rightSpeed);
  }

  public void curvatureDrive(double speed, double rotation, boolean quickturn){
    m_drive.curvatureDrive(speed, rotation, quickturn);
  }

  public void resetEncoders() {
    leftMasterMotor.setSelectedSensorPosition(0);
    rightMasterMotor.setSelectedSensorPosition(0);
  }

  
  public void zeroSensors() {
    resetEncoders();
    RobotContainer.m_hopper.resetDirection();
  }

  public double getLeftDistance() {
    return -1 * leftMasterMotor.getSelectedSensorPosition()*DriveConstants.kEncoderDistancePerPulse;
  }

  public double getRightDistance() {
    return rightMasterMotor.getSelectedSensorPosition()*DriveConstants.kEncoderDistancePerPulse;
  }

  public double getAverageDistance() {
    return (getLeftDistance() + getRightDistance()) / 2;
  }

  public double getLeftSpeed() {
    return leftMasterMotor.getSelectedSensorVelocity();
  }

  public double getRightSpeed() {
    return rightMasterMotor.getSelectedSensorVelocity();
  }

  


}
