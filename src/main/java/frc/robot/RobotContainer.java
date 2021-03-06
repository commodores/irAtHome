// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToTarget;
import frc.robot.commands.CalculatedShot;
import frc.robot.commands.DefenseTrench;
import frc.robot.commands.DriveManual;
import frc.robot.commands.SimpleShoot;
import frc.robot.commands.SixBallAuto;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.VelocityShooter;
import edu.wpi.first.wpilibj2.command.Command;
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
  public static final Climber m_Climber = new Climber();
  public static final Intake m_intake = new Intake();
  //public static final Shooter m_shooter = new Shooter();
  public static final Feeder m_feeder = new Feeder();
  public static final VelocityShooter m_shooter = new VelocityShooter();
  public static final Hopper m_hopper = new Hopper();
  public static final Compressor m_compressor = new Compressor();
  public static final LimeLight m_limelight = new LimeLight();
  
  public static final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  public static final XboxController m_driver2Controller = new XboxController(OIConstants.kDriverController2Port);

  private final SendableChooser<String> m_autoChooser = new SendableChooser<>();
  
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

    //Shooter

    new JoystickButton(m_driverController, Button.kX.value)
      .whenPressed(() -> m_feeder.runFeed(.5))
      .whenReleased(() -> m_feeder.stopFeeder());

    new JoystickButton(m_driverController, Button.kY.value)
      .whenPressed(() -> m_feeder.runFeed(-.35))
      .whenReleased(() -> m_feeder.stopFeeder());

    new JoystickButton(m_driverController, Button.kBumperLeft.value)
      .whenPressed(() -> m_shooter.setRPM(1900))
      .whenReleased(() -> m_shooter.setRPM(-1));

    new JoystickButton(m_driverController, Button.kBumperRight.value)
      .whenPressed(() -> m_shooter.setRPM(2000))
      .whenReleased(() -> m_shooter.setRPM(-1));

    new JoystickButton(m_driverController, Button.kStart.value)
      .whileHeld(new CalculatedShot());

    new JoystickButton(m_driver2Controller, Button.kBack.value)
      .whenPressed(()-> m_shooter.setRPM(1000));

    new JoystickButton(m_driver2Controller, Button.kStart.value)
      .whenPressed(() -> m_shooter.setRPM(-1));
    
    //Hood
    
    new JoystickButton(m_driver2Controller, Button.kBumperLeft.value)
      .whenPressed(()-> m_shooter.hoodUp());

    new JoystickButton(m_driver2Controller, Button.kBumperRight.value)
      .whenPressed(()-> m_shooter.hoodDown());

    
    //Limelight

    new JoystickButton(m_driverController, Button.kBack.value)
    .whenPressed(new AlignToTarget());


    //Intake

    new JoystickButton(m_driver2Controller, Button.kA.value)
      .whileHeld(() -> m_intake.runIntake(-1))
      .whenReleased(() -> m_intake.stopIntake());

    new JoystickButton(m_driver2Controller, Button.kB.value)
    .whileHeld(() -> m_intake.runIntake(1))
    .whenReleased(() -> m_intake.stopIntake());

    new JoystickButton(m_driverController, Button.kA.value)
      .whenPressed(() -> m_intake.extendIntake());

    new JoystickButton(m_driverController, Button.kB.value)
      .whenPressed(() -> m_intake.retractIntake());


    //Hopper

    new JoystickButton(m_driver2Controller, Button.kX.value)
      .whenPressed(() -> m_hopper.runHopper(.35))
      .whenReleased(() -> m_hopper.stopHopper());

    new JoystickButton(m_driver2Controller, Button.kY.value)
      .whenPressed(() -> m_hopper.runHopper(-.35))
      .whenReleased(() -> m_hopper.stopHopper());

    //Climber

    
    
      
  }

  private void initializeStartup()
  {
    
    //SmartDashboard.putData("Ramp it up!!", new AutoShoot());
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
    m_autoChooser.addOption("3 Ball Auto", "threeball");
    m_autoChooser.addOption("6 Ball Auto", "sixball");
    m_autoChooser.addOption("5 Ball Auto", "SecondAuto");

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
      case "threeball":
        return new SimpleShoot();
        //RobotContainer.m_drivetrain.setPos(Units.feetToMeters(3.5), Units.feetToMeters(-5));
        //return new RunTrajectory(getSlalom());
      case "sixball" :
        return new SixBallAuto();
      
      case "SecondAuto" :
        return new DefenseTrench();
    
      default:
        System.out.println("\nError selecting autonomous command:\nCommand selected: " + m_autoChooser.getSelected() + "\n");
        return null;
    }
    
  }
}