// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private final CANSparkMax leftIntake = new CANSparkMax(IntakeConstants.kLeftIntakePort, MotorType.kBrushless);
  private final CANSparkMax rightIntake = new CANSparkMax(IntakeConstants.kRightIntakePort, MotorType.kBrushless);

  

  private final WPI_TalonSRX intakeMotor;
  private final Solenoid intakeSolenoid;

  public Intake() {

    intakeMotor = new WPI_TalonSRX(IntakeConstants.kIntakePort);

    intakeSolenoid = new Solenoid(IntakeConstants.kIntakeSolenoidPort);


    intakeMotor.configFactoryDefault();
    leftIntake.restoreFactoryDefaults();
    rightIntake.restoreFactoryDefaults();

    leftIntake.setOpenLoopRampRate(1.0);
    rightIntake.setOpenLoopRampRate(1.0);

    leftIntake.setIdleMode(IdleMode.kCoast);
    rightIntake.setIdleMode(IdleMode.kCoast);

    leftIntake.setInverted(true);

    intakeMotor.setNeutralMode(NeutralMode.Coast);

    intakeMotor.set(ControlMode.PercentOutput, 0.0);
    
    rightIntake.follow(leftIntake);
  

  }

  public void BallIn(){
    intakeMotor.set(ControlMode.PercentOutput, -1);
    leftIntake.set(-1);
    //rightIntake.set(-1);
  }

  public void BallOut(){
    intakeMotor.set(ControlMode.PercentOutput, .6);
    leftIntake.set(.6);
    //rightIntake.set(.6);
  }

  public void stopIntake(){
    intakeMotor.set(ControlMode.PercentOutput, 0.0);
    leftIntake.set(0.0);
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
