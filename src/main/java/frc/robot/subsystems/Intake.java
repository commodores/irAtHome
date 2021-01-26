// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final WPI_TalonSRX intakeMotor;
  private final Solenoid intakeSolenoid;

  public Intake() {

    intakeMotor = new WPI_TalonSRX(IntakeConstants.kIntakePort);
    intakeSolenoid = new Solenoid(IntakeConstants.kIntakeSolenoidPort);

    intakeMotor.configFactoryDefault();

    intakeMotor.setNeutralMode(NeutralMode.Coast);

    intakeMotor.set(ControlMode.PercentOutput, 0.0);

  }

  public void BallIn(){
    intakeMotor.set(ControlMode.PercentOutput, -1);
  }

  public void BallOut(){
    intakeMotor.set(ControlMode.PercentOutput, .6);
  }

  public void stopIntake(){
    intakeMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void extendIntake(){
    intakeSolenoid.set(true);
  }

  public void retractIntake(){
    intakeSolenoid.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
