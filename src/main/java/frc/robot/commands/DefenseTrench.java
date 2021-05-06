// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DefenseTrench extends SequentialCommandGroup {
  /** Creates a new DefenseTrench. */
  public DefenseTrench() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(
      new AlignToTarget(),
      new AutoShoot().withTimeout(2),
      new ParallelCommandGroup(
        new AutoShoot(),
        new AutoHopper(),
        new AutoFeeder()).withTimeout(1.5),
      new StopFeederAuto().withTimeout(.1),
      new StopHopperAuto().withTimeout(.1),
      new AutoTurn(-30),
      new ParallelCommandGroup(
        new FiveBallAuto(),
        new AutoIntake()).withTimeout(5.5),
      new StopIntake().withTimeout(.1),
      new AutoTurn(-100),
      new AlignToTarget().withTimeout(1),
      new AutoShoot().withTimeout(2),
      new ParallelCommandGroup(
        new AutoShoot(),
        new AutoHopper(),
        new AutoFeeder()).withTimeout(1.5),
      new StopFeederAuto().withTimeout(.1),
      new StopHopperAuto().withTimeout(.1)
      

    



    );
    
  }
}
