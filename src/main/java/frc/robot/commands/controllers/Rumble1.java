// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.controllers;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.j;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Rumble1 extends SequentialCommandGroup {
  /** Creates a new Rumble1S. */
  public Rumble1() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(new WaitCommand(.1), new RepeatCommand(new InstantCommand(() -> j.driverRumble.setRumble(RumbleType.kBothRumble,1))), new RepeatCommand(new InstantCommand(() -> j.operator.setRumble(RumbleType.kBothRumble, 1)))),
      new InstantCommand(() -> j.driverRumble.setRumble(RumbleType.kBothRumble, 0))
    );
  }
}
