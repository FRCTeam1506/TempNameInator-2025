// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.macros;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeL3Auto extends SequentialCommandGroup {
  /** Creates a new AlgaeL3. */
  public AlgaeL3Auto(Algae algae, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    //auto version has no elevator l2 because it happens in the previous command
    addCommands(
      new InstantCommand(() -> algae.intake()).withDeadline(new WaitCommand(0.3)),
      //new InstantCommand(() -> elevator.elevatorL2()).withDeadline(new WaitCommand(0.7)),
      new InstantCommand(() -> elevator.elevatorUp()).withDeadline(new WaitCommand(0.3)),
      new InstantCommand(() -> elevator.elevatorL3Algae()).withDeadline(new WaitCommand(0.5))
    );
  }
}
