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
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToTarget;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.BlueZone;
import frc.robot.commands.DriveManual;
import frc.robot.commands.RunTrajectory;
import frc.robot.commands.GreenZone;
import frc.robot.commands.KillSwitch;
import frc.robot.commands.QuickShot;
import frc.robot.commands.RedZone;
import frc.robot.commands.YellowZone;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.VelocityShooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

  // Create config for trajectory
  TrajectoryConfig configBackwards =
    new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
                        DriveConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        .setReversed(true)
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
      .whenPressed(() -> m_shooter.setRPM(1850))
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
      .whenPressed(() -> m_hopper.runFeed(.75))
      .whenReleased(() -> m_hopper.stopFeeder());

    new JoystickButton(m_driverController, Button.kY.value)
      .whenPressed(() -> m_hopper.runFeed(-.6))
      .whenReleased(() -> m_hopper.stopFeeder());

    new JoystickButton(rightJoystick, 4)
      .whenPressed(()  -> m_shooter.blueAlt());

    new JoystickButton(rightJoystick, 5)
      .whenPressed(() -> m_hopper.runHopper(1))
      .whenReleased(() -> m_hopper.stopHopper());

    new JoystickButton(rightJoystick, 6)
      .whenPressed(() -> m_hopper.runHopper(-1))
      .whenReleased(() -> m_hopper.stopHopper());

    new JoystickButton(rightJoystick, 2)
      .whileHeld(() -> m_intake.runIntake(.8))
      .whenReleased(() -> m_intake.stopIntake());

    new JoystickButton(rightJoystick, 3)
    .whileHeld(() -> m_intake.runIntake(-.8))
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
    .whenPressed(new AlignToTarget());

    
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
    m_autoChooser.addOption("Bounce Path", "bounce");

    /* Display chooser on SmartDashboard for operators to select which autonomous command to run during the auto period. */
    SmartDashboard.putData("Autonomous Command", m_autoChooser);
  }  
  
  public Trajectory getSlalom(){
    Trajectory slalom = TrajectoryGenerator.generateTrajectory(
      // Start
      new Pose2d(Units.feetToMeters(3.5), Units.feetToMeters(-5), new Rotation2d(0)),
      List.of(
          new Translation2d(Units.feetToMeters(7.5), Units.feetToMeters(-2.5)),
          new Translation2d(Units.feetToMeters(10), Units.feetToMeters(-.5)),
          new Translation2d(Units.feetToMeters(13), Units.feetToMeters(.5)),
          new Translation2d(Units.feetToMeters(17), Units.feetToMeters(.5)),
          new Translation2d(Units.feetToMeters(20), Units.feetToMeters(-.5)),
          new Translation2d(Units.feetToMeters(22.5), Units.feetToMeters(-2.5)),
          new Translation2d(Units.feetToMeters(25), Units.feetToMeters(-5.5)),
          new Translation2d(Units.feetToMeters(29), Units.feetToMeters(-2.5)),
          new Translation2d(Units.feetToMeters(26), Units.feetToMeters(.5)),
          new Translation2d(Units.feetToMeters(22.5), Units.feetToMeters(-2.5)),
          new Translation2d(Units.feetToMeters(20), Units.feetToMeters(-6.5)),
          new Translation2d(Units.feetToMeters(17), Units.feetToMeters(-7)),
          new Translation2d(Units.feetToMeters(13), Units.feetToMeters(-7)),
          new Translation2d(Units.feetToMeters(7.5), Units.feetToMeters(-2.5))

      ),
      new Pose2d(Units.feetToMeters(5), Units.feetToMeters(2.5), new Rotation2d(Math.PI / 2)),
      config
    );
    return slalom;
  }

  public Trajectory getBarrelRacing(){
    Trajectory barrel = TrajectoryGenerator.generateTrajectory(
      // Start
      new Pose2d(Units.feetToMeters(2.5), Units.feetToMeters(0), new Rotation2d(0)), 
      List.of(
        
        new Translation2d(Units.feetToMeters(12.5),Units.feetToMeters(0)),//2
        new Translation2d(Units.feetToMeters(15.5),Units.feetToMeters(-3)),//3
        new Translation2d(Units.feetToMeters(12),Units.feetToMeters(-5.5)),//4
        new Translation2d(Units.feetToMeters(8),Units.feetToMeters(-2)),//5
        new Translation2d(Units.feetToMeters(12),Units.feetToMeters(.5)),//6
        new Translation2d(Units.feetToMeters(19),Units.feetToMeters(0)),//7
        new Translation2d(Units.feetToMeters(22),Units.feetToMeters(3.5)),//8
        new Translation2d(Units.feetToMeters(18),Units.feetToMeters(6)),//9
        new Translation2d(Units.feetToMeters(16),Units.feetToMeters(1.5)),//10
        new Translation2d(Units.feetToMeters(22),Units.feetToMeters(-5)),//11
        new Translation2d(Units.feetToMeters(26),Units.feetToMeters(-6.5)),//12
        new Translation2d(Units.feetToMeters(28),Units.feetToMeters(-4)),//13
        new Translation2d(Units.feetToMeters(24),Units.feetToMeters(-.5))//14
        

      ),
    new Pose2d(Units.feetToMeters(-1), Units.feetToMeters(-.5), new Rotation2d(Math.PI)), 
    config);
    return barrel;
  }

  public Trajectory getBounce1(){
    Trajectory bounce1 = TrajectoryGenerator.generateTrajectory(
      // Start
      new Pose2d(Units.feetToMeters(2.5), Units.feetToMeters(0), new Rotation2d(0)), 
      List.of(
        new Translation2d(Units.feetToMeters(6),Units.feetToMeters(1))
      ),
    new Pose2d(Units.feetToMeters(7.5), Units.feetToMeters(3.5), new Rotation2d(-80)), 
    config);
    return bounce1;
  }

  public Trajectory getBounce2(){
    Trajectory bounce2 = TrajectoryGenerator.generateTrajectory(
      // Start
      new Pose2d(Units.feetToMeters(7.5), Units.feetToMeters(3.5), new Rotation2d(-80)), 
      List.of(
        new Translation2d(Units.feetToMeters(10),Units.feetToMeters(-2)),
        new Translation2d(Units.feetToMeters(12.5), Units.feetToMeters(-5)),
        new Translation2d(Units.feetToMeters(15),Units.feetToMeters(-2))
      ),
    new Pose2d(Units.feetToMeters(15), Units.feetToMeters(5), new Rotation2d(80)), 
    configBackwards);
    return bounce2;
  }

  public Trajectory getBounce3(){
    Trajectory bounce3 = TrajectoryGenerator.generateTrajectory(
      // Start
      new Pose2d(Units.feetToMeters(15), Units.feetToMeters(5), new Rotation2d(80)), 
      List.of(
        new Translation2d(Units.feetToMeters(15),Units.feetToMeters(-2)),
        new Translation2d(Units.feetToMeters(19),Units.feetToMeters(-3.5)),
        new Translation2d(Units.feetToMeters(22),Units.feetToMeters(-2))
      ),
    new Pose2d(Units.feetToMeters(22.5), Units.feetToMeters(7), new Rotation2d(-80)), 
    config);
    return bounce3;
  }

  public Trajectory getBounce4(){
    Trajectory bounce4 = TrajectoryGenerator.generateTrajectory(
      // Start
      new Pose2d(Units.feetToMeters(22.5), Units.feetToMeters(7), new Rotation2d(90)), 
      List.of(
        //new Translation2d(Units.feetToMeters(22.5),Units.feetToMeters(3))
      ),
    new Pose2d(Units.feetToMeters(30), Units.feetToMeters(5), new Rotation2d(180)), 
    configBackwards);
    return bounce4;
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
        return new RunTrajectory(getSlalom());
      case "barrel":
        RobotContainer.m_drivetrain.setPos(Units.feetToMeters(2.5), 0);
        return new RunTrajectory(getBarrelRacing());
      case "bounce":
        RobotContainer.m_drivetrain.setPos(Units.feetToMeters(2.5), 0);
        return new SequentialCommandGroup(new RunTrajectory(getBounce1()), new RunTrajectory(getBounce2()), new RunTrajectory(getBounce3()), new RunTrajectory(getBounce4()));
      default:
        System.out.println("\nError selecting autonomous command:\nCommand selected: " + m_autoChooser.getSelected() + "\n");
        return null;
    }
    
  }
}