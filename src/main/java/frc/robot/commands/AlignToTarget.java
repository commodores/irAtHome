// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.LimeLight;

public class AlignToTarget extends CommandBase {
  double Kp = 0.009;
  double min_command = 1;

  DriveTrain m_DriveTrain;
  LimeLight m_LimeLight;
  Hopper m_Hopper;

    public AlignToTarget() {
        // Use requires() here to declare subsystem dependencies
        addRequirements(m_DriveTrain);
        addRequirements(m_LimeLight);
        addRequirements(m_Hopper);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {

      double heading_error = -m_LimeLight.getXAngle();
      double steering_adjust = 0.0;
      
      if (m_LimeLight.getXAngle() > 1.0)
      {
              steering_adjust = Kp*heading_error - min_command;
      }
      else if (m_LimeLight.getXAngle() < 1.0)
      {
              steering_adjust = Kp*heading_error + min_command;
      }
      double left_command = -steering_adjust;
      double right_command = steering_adjust;

      m_DriveTrain.tankDriveVolts(left_command, right_command);
     
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
      return m_LimeLight.getXAngle() < .5 && m_LimeLight.getXAngle() > -.5;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
      m_DriveTrain.tankDriveVolts(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    
}
