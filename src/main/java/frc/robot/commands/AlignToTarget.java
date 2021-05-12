// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AlignToTarget extends CommandBase {
  double Kp = 0.009;
  double min_command = 1;

  public AlignToTarget() {
        // Use requires() here to declare subsystem dependencies
        addRequirements(RobotContainer.m_drivetrain);
        addRequirements(RobotContainer.m_limelight);
        addRequirements(RobotContainer.m_hopper);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {

      double heading_error = - RobotContainer.m_limelight.getXAngle();
      double steering_adjust = 0.0;
      
      if (RobotContainer.m_limelight.getXAngle() > 1.0)
      {
              steering_adjust = Kp*heading_error - min_command;
      }
      else if (RobotContainer.m_limelight.getXAngle() < 1.0)
      {
              steering_adjust = Kp*heading_error + min_command;
      }
      double left_command = -steering_adjust;
      double right_command = steering_adjust;

      RobotContainer.m_drivetrain.tankDriveVolts(left_command, right_command);
     
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
      return RobotContainer.m_limelight.getXAngle() < .5 && RobotContainer.m_limelight.getXAngle() > -.5;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
      RobotContainer.m_drivetrain.tankDriveVolts(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    
}
