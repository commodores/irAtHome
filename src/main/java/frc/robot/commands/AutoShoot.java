// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VelocityShooter;

public class AutoShoot extends CommandBase {
  VelocityShooter m_Shooter;
  /** Creates a new AutoShoot. */
  public AutoShoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<< HEAD
    RobotContainer.m_shooter.setRPM(2100);
=======
    m_Shooter.setRPM(2100);
>>>>>>> fa7843ad33fbfbc090897a4e3ad9494d6f2b8bd9
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Shooter.setRPM(-1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
