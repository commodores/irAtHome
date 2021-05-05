/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class AutoTurn extends CommandBase {
  DriveTrain m_DriveTrain;
  private double degrees;
  private double currentHeading;
  /**
   * Creates a new AutoTurn.
   */
  public AutoTurn(double getDegrees) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveTrain);
    degrees = getDegrees;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveTrain.resetEncoders();
    currentHeading = m_DriveTrain.getDirection();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(degrees > 0){
      m_DriveTrain.tankDriveVolts(3.0, -3.0);
    } else {
      m_DriveTrain.tankDriveVolts(-3.0, 3.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveTrain.tankDriveVolts(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(degrees > 0){
      //turn right
      return currentHeading - m_DriveTrain.getDirection() > degrees;
    } else {
      //turn left
      return currentHeading - m_DriveTrain.getDirection() < degrees;
    }
  }
}