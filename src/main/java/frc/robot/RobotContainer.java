// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToTarget;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.BlueZone;
import frc.robot.commands.DriveManual;
import frc.robot.commands.FwdTrajectory;
import frc.robot.commands.GreenZone;
import frc.robot.commands.KillSwitch;
import frc.robot.commands.QuickShot;
import frc.robot.commands.RedZone;
import frc.robot.commands.YellowZone;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;
//import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VelocityShooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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
  public static final DriveTrain m_drivetrain = new DriveTrain();
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
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    /* Initialize various systems on robotInit. */
    this.initializeStartup();

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
      .whenPressed(() -> m_shooter.setRPM(1900))
      .whenReleased(() -> m_shooter.setRPM(-1));

    //new JoystickButton(m_driverController, Button.kBack.value)
    //  .whenPressed(() -> m_shooter.setRPM(2350))
    //  .whenReleased(() -> m_shooter.setRPM(-1));

    //new JoystickButton(m_driverController, Button.kStart.value)
    //  .whenPressed(() -> m_shooter.setRPM(3000))
    //  .whenReleased(() -> m_shooter.setRPM(-1));

      new JoystickButton(m_driverController, Button.kBumperRight.value)
      .whileHeld(() -> m_shooter.setRPM(2000))
      .whenReleased(() -> m_shooter.setRPM(-1));

    new JoystickButton(m_driverController, Button.kX.value)
      .whenPressed(() -> m_hopper.runFeed(.45))
      .whenReleased(() -> m_hopper.stopFeeder());

    new JoystickButton(m_driverController, Button.kY.value)
      .whenPressed(() -> m_hopper.runFeed(-.45))
      .whenReleased(() -> m_hopper.stopFeeder());

    new JoystickButton(rightJoystick, 4)
      .whenPressed(()  -> m_shooter.blueAlt());

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
      .whenPressed(new AlignToTarget());

    new JoystickButton(m_driverController, Button.kBack.value)
      .whenPressed(new AutoDrive(2.95));

    new JoystickButton(m_driverController, Button.kStart.value)
    .whenPressed(new AutoDrive(-2.95));
  }

  private void initializeStartup()
  {

    SmartDashboard.putData("Green Zone", new GreenZone());
    SmartDashboard.putData("Yellow Zone", new YellowZone());
    SmartDashboard.putData("Blue Zone", new BlueZone());
    SmartDashboard.putData("Red Zone", new RedZone());
    SmartDashboard.putData("Quick Shot", new QuickShot());
    SmartDashboard.putData("Kill Switch", new KillSwitch().withTimeout(.0001));

    m_drivetrain.setDefaultCommand(
      new DriveManual(m_drivetrain));
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
    m_autoChooser.setDefaultOption("Do Nothing", "doNothing");
    m_autoChooser.addOption("Slalom", "slalom");
    m_autoChooser.addOption("Barrel Racing", "barrel");

    /* Display chooser on SmartDashboard for operators to select which autonomous command to run during the auto period. */
    SmartDashboard.putData("Autonomous Command", m_autoChooser);
  }  
  
  public Trajectory getSlalom(){
    Trajectory slalom = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(
          new Translation2d(.75, .25),
          new Translation2d(1, 1),
          new Translation2d(1.75, 1.75),
          new Translation2d(4.75, 1.75),
          new Translation2d(5.5, 1.25),
          new Translation2d(6, .5),
          new Translation2d(6.75, 0),
          new Translation2d(7.5, .85),
          new Translation2d(6.75, 1.5),
          new Translation2d(6, 1),
          new Translation2d(5.5, -.25),
          new Translation2d(4.75, -.5),
          new Translation2d(1.75, -.5),
          new Translation2d(1.25, -.25)
          //new Translation2d(.65, .65)
          //new Translation2d(.25, .65)
          //new Translation2d(.25, .75)

      ),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(.75, .65, new Rotation2d(m_drivetrain.getDirection())),
      // Pass config
      config
    );
    return slalom;
  }

  public Trajectory getSlalomTest(){
    Trajectory slalom = TrajectoryGenerator.generateTrajectory(
      // Start
      new Pose2d(Units.feetToMeters(3.5), Units.feetToMeters(-5), new Rotation2d(0)),
      List.of(
          new Translation2d(Units.feetToMeters(7.56), Units.feetToMeters(-1.53)),
          new Translation2d(Units.feetToMeters(12.78), Units.feetToMeters(.33)),
          new Translation2d(Units.feetToMeters(18.54), Units.feetToMeters(.42)),
          new Translation2d(Units.feetToMeters(22.44), Units.feetToMeters(-2.55)),
          new Translation2d(Units.feetToMeters(25.35), Units.feetToMeters(-3.66)),
          new Translation2d(Units.feetToMeters(29.49), Units.feetToMeters(.66)),
          new Translation2d(Units.feetToMeters(23.52), Units.feetToMeters(1.83)),
          new Translation2d(Units.feetToMeters(19.98), Units.feetToMeters(-5.1)),
          new Translation2d(Units.feetToMeters(9.96), Units.feetToMeters(-5.01))
      ),
      new Pose2d(Units.feetToMeters(5.34), Units.feetToMeters(-.33), new Rotation2d(Math.PI / 2)),
      config
    );
    return slalom;
  }

  public Trajectory getBarrelRacing(){
    Trajectory barrel = TrajectoryGenerator.generateTrajectory(
      // Start
      new Pose2d(1.143, 1.905, new Rotation2d(0)), 
      List.of(
        new Translation2d(1.651,2.032),
        new Translation2d(3.048,2.032), 
        new Translation2d(4.064,1.905),
        new Translation2d(4.318,1.524),
        new Translation2d(3.302,1.524),
        new Translation2d(4.064,1.905),
        new Translation2d(5.842,2.159),
        new Translation2d(6.858,3.048),
        new Translation2d(6.604,3.683),
        new Translation2d(5.588,3.302),
        new Translation2d(5.842,2.159),
        new Translation2d(6.858,1.016),
        new Translation2d(8.001,1.270),
        new Translation2d(8.255,1.651),
        new Translation2d(6.858,2.032),
        new Translation2d(5.842,2.159),
        new Translation2d(4.064,2.540)
      ),
    new Pose2d(1.143, 1.905, new Rotation2d(Math.PI)), 
    config);
    return barrel;
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    switch (m_autoChooser.getSelected())
    {
      case "slalom":
        RobotContainer.m_drivetrain.setPos(Units.feetToMeters(3.5), Units.feetToMeters(-5));
        return new FwdTrajectory(getSlalomTest());
      case "barrel":
        RobotContainer.m_drivetrain.setPos(1.143, 1.905);
        return new FwdTrajectory(getBarrelRacing());
      default:
        System.out.println("\nError selecting autonomous command:\nCommand selected: " + m_autoChooser.getSelected() + "\n");
        return null;
    }
    
  }
}
    
    



