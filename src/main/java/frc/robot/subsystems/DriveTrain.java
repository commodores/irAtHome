// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

  //Set Electronics To Follow Mode
    rightSlaveMotor.follow(rightMasterMotor);

    leftSlaveMotor.follow(leftMasterMotor);
    
  //Set Electronics To Brake Mode
    rightMasterMotor.setNeutralMode(NeutralMode.Coast);

    leftMasterMotor.setNeutralMode(NeutralMode.Coast);
  
  //Set Electronics To Follow Control Mode
    rightMasterMotor.set(ControlMode.PercentOutput, 0.0);

    leftMasterMotor.set(ControlMode.PercentOutput, 0.0);
  
  //Setting Default Command
    setDefaultCommand(new DriveManual(this));
  
  // Finalizing DriveTrain
     m_drive = new DifferentialDrive(leftMasterMotor, rightMasterMotor);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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


}
