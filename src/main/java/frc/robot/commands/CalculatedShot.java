// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.VelocityShooter;

public class CalculatedShot extends CommandBase {

  LimeLight m_LimeLight;
  VelocityShooter m_Shooter;

  double distance, minOutput, maxOutput, minDistance, maxDistance, slope;

  /** Creates a new CalculatedShot. */
  public CalculatedShot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_LimeLight);
    addRequirements(m_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
<<<<<<< HEAD
    distance = RobotContainer.m_limelight.getDistance();
    minOutput = 1925;
    maxOutput = 2050;
=======
    distance = m_LimeLight.getDistance();
    minOutput = 1950;
    maxOutput = 2100;
>>>>>>> fa7843ad33fbfbc090897a4e3ad9494d6f2b8bd9
    minDistance = 140;
    maxDistance = 220;
    slope = (maxOutput - minOutput) / (maxDistance - minDistance);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double setSpeed = minOutput + Math.round(slope * (distance - minDistance));

    m_Shooter.setRPM(setSpeed);

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
