// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.controller.PIDController;

public class AutoDrive extends SequentialCommandGroup {
  private double distance;

  // Create a voltage constraint to ensure we don't accelerate too fast
  DifferentialDriveVoltageConstraint autoVoltageConstraint =
  new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                 DriveConstants.kvVoltSecondsPerMeter,
                                 DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics,
      10);
  // Create config for trajectory
  TrajectoryConfig config =
    new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                        DriveConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

  RamseteCommand ramseteCommand;

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ramseteCommand.isFinished();
  }

  /** Creates a new AutoDrive. */
  public AutoDrive(double getDistance) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_drivetrain);
    distance = getDistance;

    if(distance < 0){
      config.setReversed(true);
      }
      
    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            //new Translation2d(1, 0),
            //new Translation2d(2, 0)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d((distance<0)?-distance:distance, 0, new Rotation2d(0)),
        // Pass config
        config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        RobotContainer.m_drivetrain::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        RobotContainer.m_drivetrain::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        RobotContainer.m_drivetrain::tankDriveVolts,
        RobotContainer.m_drivetrain
    );

    // Reset odometry to the starting pose of the trajectory.
    RobotContainer.m_drivetrain.zeroSensors();

    // Run path following command, then stop at the end.
    addCommands(
      ramseteCommand.andThen(() -> RobotContainer.m_drivetrain.tankDriveVolts(0, 0))
    );

    
  }

}
