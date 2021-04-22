// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.*;
import frc.robot.RobotContainer;

public class ClimberManual extends CommandBase {
  /** Creates a new ClimberManual. */

  
  
  private final Climber m_Climber;
  public ClimberManual(Climber Climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_Climber = Climber;
    addRequirements(Climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.rightJoystick.getRawAxis(1) > 0.5){
      m_Climber.ClimberUp();
    } else if (RobotContainer.rightJoystick.getRawAxis(1) < -0.5){
      m_Climber.ClimberDown();
    }else {
      m_Climber.StopClimber();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
