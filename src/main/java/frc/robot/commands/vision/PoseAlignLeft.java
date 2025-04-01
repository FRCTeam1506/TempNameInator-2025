// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PoseAlignLeft extends Command {
  /** Creates a new PoseAlignLeft. */
  int id; CommandSwerveDrivetrain drivetrain;
  public PoseAlignLeft(CommandSwerveDrivetrain drivetrain, int id) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.id = id;
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
            if (id != -1)
            drivetrain.pathPIDTo(CommandSwerveDrivetrain.tagPoseAndymarkMap.get(id).transformBy(VisionConstants.leftBranch), CommandSwerveDrivetrain.tagPoseAndymarkMap.get(id));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


}
