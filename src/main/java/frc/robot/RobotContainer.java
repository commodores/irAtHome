// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.annotation.meta.When;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
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
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public static final DriveTrain m_drivetrain = new DriveTrain();
  public static final Intake m_intake = new Intake();
  public static final Shooter m_shooter = new Shooter();
  public static final Hopper m_hopper = new Hopper();
  public static final Compressor m_compressor = new Compressor();
  
  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  public static final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  public static final Joystick leftJoystick = new Joystick(OIConstants.kLeftJoystickPort);
  public static final Joystick rightJoystick = new Joystick(OIConstants.kRightJoystickPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    new JoystickButton(m_driverController, Button.kA.value)
      //.whenPressed(m_intake.extendIntake())
      .whenPressed(() -> m_intake.extendIntake());

    new JoystickButton(m_driverController, Button.kB.value)
      .whenPressed(() -> m_intake.retractIntake());

    new JoystickButton(m_driverController, Button.kBumperLeft.value)
      .whileHeld(() -> m_shooter.set(.35))
      .whenReleased(()-> m_shooter.stop());

    new JoystickButton(m_driverController, Button.kBumperRight.value)
      .whileHeld(() -> m_shooter.set(.45))
      .whenReleased(()-> m_shooter.stop());

    new JoystickButton(m_driverController, Button.kX.value)
      .whenPressed(() -> m_hopper.feedIn())
      .whenReleased(() -> m_hopper.stopFeeder());

    new JoystickButton(m_driverController, Button.kY.value)
      .whenPressed(() -> m_hopper.feedOut())
      .whenReleased(() -> m_hopper.stopFeeder());

    new JoystickButton(m_driverController, Button.kBack.value)
      .whenPressed(() -> m_hopper.hopperIn())
      .whenReleased(() -> m_hopper.stopHopper());

    new JoystickButton(m_driverController, Button.kStart.value)
      .whenPressed(() -> m_hopper.hopperOut())
      .whenReleased(() -> m_hopper.stopHopper());

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
