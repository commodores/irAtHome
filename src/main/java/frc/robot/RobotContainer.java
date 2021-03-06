// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToTarget;
import frc.robot.commands.AutoDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.NewDrive;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;
//import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VelocityShooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //public static final DriveTrain m_drivetrain = new DriveTrain();
  public static final NewDrive m_drivetrain = new NewDrive();
  public static final Intake m_intake = new Intake();
  //public static final Shooter m_shooter = new Shooter();
  public static final VelocityShooter m_shooter = new VelocityShooter();
  public static final Hopper m_hopper = new Hopper();
  public static final Compressor m_compressor = new Compressor();
  public static final LimeLight m_limelight = new LimeLight();
  
  public static final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  public static final Joystick leftJoystick = new Joystick(OIConstants.kLeftJoystickPort);
  public static final Joystick rightJoystick = new Joystick(OIConstants.kRightJoystickPort);

  private final SendableChooser<String> m_autoChooser = new SendableChooser<>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    /* Initialize autonomous command chooser and display on the SmartDashboard. */
    this.initializeAutoChooser();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_driverController, Button.kA.value)
      .whenPressed(() -> m_intake.extendIntake());

    new JoystickButton(m_driverController, Button.kB.value)
      .whenPressed(() -> m_intake.retractIntake());

    new JoystickButton(m_driverController, Button.kBumperLeft.value)
      .whenPressed(() -> m_shooter.setRPM(2000))
      .whenReleased(() -> m_shooter.setRPM(1000));

    new JoystickButton(m_driverController, Button.kBack.value)
      .whenPressed(() -> m_shooter.setRPM(2350))
      .whenReleased(() -> m_shooter.setRPM(-1));

    new JoystickButton(m_driverController, Button.kStart.value)
      .whenPressed(() -> m_shooter.setRPM(3000))
      .whenReleased(() -> m_shooter.setRPM(-1));

      new JoystickButton(m_driverController, Button.kBumperRight.value)
      .whileHeld(() -> m_shooter.setRPM(4400))
      .whenReleased(() -> m_shooter.setRPM(-1));

    new JoystickButton(m_driverController, Button.kX.value)
      .whenPressed(() -> m_hopper.runFeed(.55))
      .whenReleased(() -> m_hopper.stopFeeder());

    new JoystickButton(m_driverController, Button.kY.value)
      .whenPressed(() -> m_hopper.runFeed(-.55))
      .whenReleased(() -> m_hopper.stopFeeder());

    new JoystickButton(rightJoystick, 5)
      .whenPressed(() -> m_hopper.runHopper(.5))
      .whenReleased(() -> m_hopper.stopHopper());

    new JoystickButton(rightJoystick, 6)
      .whenPressed(() -> m_hopper.runHopper(-.5))
      .whenReleased(() -> m_hopper.stopHopper());

    new JoystickButton(rightJoystick, 2)
      .whileHeld(() -> m_intake.runIntake(.5))
      .whenReleased(() -> m_intake.stopIntake());

    new JoystickButton(rightJoystick, 3)
    .whileHeld(() -> m_intake.runIntake(-.5))
    .whenReleased(() -> m_intake.stopIntake());

    new JoystickButton(rightJoystick, 7)
      .whenPressed(()-> m_shooter.LongShot());

    new JoystickButton(rightJoystick, 8)
      .whenPressed(() -> m_shooter.whiteLineExtend());

    new JoystickButton(rightJoystick, 9)
      .whenPressed(()-> m_shooter.UnderGoal());

    new JoystickButton(rightJoystick, 1)
    .whenPressed(new AlignToTarget())
    .whenReleased(() -> m_drivetrain.tankDriveVolts(0.0, 0.0));

    new JoystickButton(rightJoystick, 4)
    .whenPressed(new AutoDrive(-2,.5));
  }

  /**
   * Set options for autonomous command chooser and display them for selection on the SmartDashboard.
   * Using string chooser rather than command chooser because if using a command chooser, will instantiate
   * all the autonomous commands. This may cause problems (e.g. initial trajectory position is from a
   * different command's path).
   */
  private void initializeAutoChooser()
  {
    /* Add options (which autonomous commands can be selected) to chooser. */
    m_autoChooser.setDefaultOption("Just Choot 'em'", "simpleShoot");
    m_autoChooser.addOption("Drive forward off line", "forward1");
    m_autoChooser.addOption("Drive reverse off line", "reverse1");
    m_autoChooser.addOption("6 Balls is nuts!!!", "sixBall");
    //m_autoChooser.addOption("Three Ball", "threeBall");
    //m_autoChooser.addOption("Six Ball", "sixBall");

    /* Display chooser on SmartDashboard for operators to select which autonomous command to run during the auto period. */
    SmartDashboard.putData("Autonomous Command", m_autoChooser);
  }  
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    switch (m_autoChooser.getSelected())
    {
      case "forward1":
        return new AutoDrive(1,.5)
        .withTimeout(5);
      default:
        System.out.println("\nError selecting autonomous command:\nCommand selected: " + m_autoChooser.getSelected() + "\n");
        return null;
    }
  }

  public RamseteCommand createTrajectoryCommand(Pose2d startPose, List<Translation2d> translationList, Pose2d endPose, boolean isReversed, double maxSpeedMetersPerSecond, double maxAccelerationMetersPerSecondSquared) {
    DifferentialDriveVoltageConstraint autoVoltageConstraint;
    TrajectoryConfig config;
  
    // Create a voltage constraint to ensure we don't accelerate too fast
    autoVoltageConstraint = new DifferentialDriveVoltageConstraint(m_drivetrain.getFeedforward(), DriveConstants.kDriveKinematics, 6);

    // Create config for trajectory
    config = new TrajectoryConfig(maxSpeedMetersPerSecond, maxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint)
        .setReversed(isReversed);

    var initialTime = System.nanoTime();

    // trajectory to follow. All units in meters.
    var trajectory = TrajectoryGenerator.generateTrajectory(
        startPose,
        translationList,
        endPose,
        config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(trajectory, 
            m_drivetrain::getPose,
            new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
            m_drivetrain.getFeedforward(),
            DriveConstants.kDriveKinematics,
            m_drivetrain::getWheelSpeeds,
            m_drivetrain.getLeftPidController(),
            m_drivetrain.getRightPidController(),
            m_drivetrain::tankDriveVolts,
            m_drivetrain);

    var dt = (System.nanoTime() - initialTime) / 1E6;
    System.out.println("RamseteCommand generation time: " + dt + "ms");

    // Run path following command, then stop at the end.
    return ramseteCommand;
  }

  
}
