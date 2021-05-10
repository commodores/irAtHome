/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
<<<<<<< HEAD
import frc.robot.RobotContainer;

public class AutoTurn extends CommandBase {
=======
import frc.robot.subsystems.DriveTrain;

public class AutoTurn extends CommandBase {
  DriveTrain m_DriveTrain;
>>>>>>> fa7843ad33fbfbc090897a4e3ad9494d6f2b8bd9
  private double degrees;
  private double currentHeading;
  /**
   * Creates a new AutoTurn.
   */
  public AutoTurn(double getDegrees) {
    // Use addRequirements() here to declare subsystem dependencies.
<<<<<<< HEAD
    addRequirements(RobotContainer.m_drivetrain);
=======
    addRequirements(m_DriveTrain);
>>>>>>> fa7843ad33fbfbc090897a4e3ad9494d6f2b8bd9
    degrees = getDegrees;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
<<<<<<< HEAD
    RobotContainer.m_drivetrain.resetEncoders();
    currentHeading = RobotContainer.m_drivetrain.getDirection();
=======
    m_DriveTrain.resetEncoders();
    currentHeading = m_DriveTrain.getDirection();
>>>>>>> fa7843ad33fbfbc090897a4e3ad9494d6f2b8bd9
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(degrees > 0){
<<<<<<< HEAD
      RobotContainer.m_drivetrain.tankDriveVolts(6.0, -6.0);
    } else {
      RobotContainer.m_drivetrain.tankDriveVolts(-6.0, 6.0);
=======
      m_DriveTrain.tankDriveVolts(3.0, -3.0);
    } else {
      m_DriveTrain.tankDriveVolts(-3.0, 3.0);
>>>>>>> fa7843ad33fbfbc090897a4e3ad9494d6f2b8bd9
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
<<<<<<< HEAD
    RobotContainer.m_drivetrain.tankDriveVolts(0.0, 0.0);
=======
    m_DriveTrain.tankDriveVolts(0.0, 0.0);
>>>>>>> fa7843ad33fbfbc090897a4e3ad9494d6f2b8bd9
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(degrees > 0){
      //turn right
<<<<<<< HEAD
      return currentHeading - RobotContainer.m_drivetrain.getDirection() > degrees;
    } else {
      //turn left
      return currentHeading - RobotContainer.m_drivetrain.getDirection() < degrees;
=======
      return currentHeading - m_DriveTrain.getDirection() > degrees;
    } else {
      //turn left
      return currentHeading - m_DriveTrain.getDirection() < degrees;
>>>>>>> fa7843ad33fbfbc090897a4e3ad9494d6f2b8bd9
    }
  }
}