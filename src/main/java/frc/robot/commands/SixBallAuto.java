// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SixBallAuto extends SequentialCommandGroup { 
  

  /** Creates a new SixBallAuto. */


  public SixBallAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    
    super(
      new AlignToTarget(),
      new AutoShoot().withTimeout(2.5),
      new ParallelCommandGroup(
        new AutoShoot(),
        new AutoHopper(),
        new AutoFeeder()).withTimeout(1.5),
      new StopFeederAuto().withTimeout(.1),
      new StopHopperAuto().withTimeout(.1),
      new StopShooterAuto().withTimeout(.1),
      new ParallelCommandGroup(
        new AutoIntake(),
        new RunTrajectory()),
      new StopIntake().withTimeout(.1),
      new AutoTurn(130),
      new AutoDrive(2).withTimeout(1),
      new AlignToTarget(),
      new AutoShot().withTimeout(1.5),
      new ParallelCommandGroup(
        new AutoHopper(),
        new AutoShot(),
        new AutoFeeder()).withTimeout(2.5),
      new StopFeederAuto().withTimeout(.1),
      new StopShooterAuto(),
      new StopHopperAuto().withTimeout(.1)
    );
    
  }

  
}
