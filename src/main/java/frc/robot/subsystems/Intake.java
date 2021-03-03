// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final CANSparkMax leftIntake = new CANSparkMax(IntakeConstants.kLeftIntakePort, MotorType.kBrushless);
  private final CANSparkMax rightIntake = new CANSparkMax(IntakeConstants.kRightIntakePort, MotorType.kBrushless);

  

  private final Solenoid intakeSolenoid;

  public Intake() {
    
    intakeSolenoid = new Solenoid(IntakeConstants.kIntakeSolenoidPort);

    leftIntake.restoreFactoryDefaults();
    rightIntake.restoreFactoryDefaults();

    leftIntake.setIdleMode(IdleMode.kCoast);
    rightIntake.setIdleMode(IdleMode.kCoast);

    leftIntake.setInverted(false);
  }

  public void BallOut(){
    leftIntake.set(-.5);
    rightIntake.set(.5);
  }

  public void BallIn(){
    leftIntake.set(.8);
    rightIntake.set(-.9);
  }

  public void stopIntake(){
    leftIntake.set(0.0);
    rightIntake.set(0.0);
  }

  public void extendIntake() {
    intakeSolenoid.set(true);
  }

  public void retractIntake() {
    intakeSolenoid.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
