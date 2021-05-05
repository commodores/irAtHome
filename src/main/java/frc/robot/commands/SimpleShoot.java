// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Feeder;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleShoot extends SequentialCommandGroup {
  /** Creates a new SimpleShoot. */
  public SimpleShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(
      new AlignToTarget(),
      new AutoShoot().withTimeout(1),
      new ParallelCommandGroup(
        new AutoShoot(),
        new AutoFeeder()).withTimeout(2),
      new StopFeederAuto().withTimeout(.1),
      new AutoHopper().withTimeout(.5),
      new StopHopperAuto().withTimeout(.1),
      new AutoShoot().withTimeout(1.5),
      new ParallelCommandGroup(
        new AutoShoot(),
        new AutoFeeder()).withTimeout(2),
      new StopShooterAuto().withTimeout(.1),
      new StopFeederAuto().withTimeout(.1),
      new AutoDrive(-1).withTimeout(3)
    );
  }
}
