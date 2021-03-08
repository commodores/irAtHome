// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AlignToTarget extends CommandBase {
  double setpoint = 0;

    public AlignToTarget() {
        // Use requires() here to declare subsystem dependencies
        addRequirements(RobotContainer.m_drivetrain);
        addRequirements(RobotContainer.m_limelight);
        addRequirements(RobotContainer.m_hopper);
    }

    // Called just before this Command runs the first time
    @Override
    public void initialize() {
      RobotContainer.m_drivetrain.zeroHeading();;
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    public void execute() {

      setpoint = RobotContainer.m_limelight.getXAngle();

      RobotContainer.m_drivetrain.tankDriveVolts(-setpoint * .1, setpoint * .1);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    public boolean isFinished() {
      return RobotContainer.m_limelight.getXAngle() < 1 && RobotContainer.m_limelight.getXAngle() > -1;
    }

    // Called once after isFinished returns true
    @Override
    public void end(boolean interrupted) {
      RobotContainer.m_drivetrain.tankDriveVolts(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    
}
