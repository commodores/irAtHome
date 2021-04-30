// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class CalculatedShot extends CommandBase {

  double distance;
  /** Creates a new CalculatedShot. */
  public CalculatedShot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_limelight);
    addRequirements(RobotContainer.m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distance = RobotContainer.m_limelight.getDistance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(distance>190){
      RobotContainer.m_shooter.setRPM(2025);
    } else {
      RobotContainer.m_shooter.setRPM(1825);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_shooter.setRPM(-1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
