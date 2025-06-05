// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pathfinding;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.vision.PoseAlign;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SmartPathfinding extends Command {
  /** Creates a new PathfindingCommand. */
  CommandSwerveDrivetrain drivetrain;
  PoseAlign poseAlign;
  public SmartPathfinding(CommandSwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    // poseAlign = new PoseAlign(drivetrain, false);
  }

  Command pathfindingCommand;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Since we are using a holonomic drivetrain, the rotation component of this pose
    // represents the goal holonomic rotation
    int face = 0;
    Pose2d targetPose;

    if(RobotContainer.autoDriveLocation.getSelected() != null){
      face = RobotContainer.autoDriveLocation.getSelected();
    }

    int id = 7;
    if(DriverStation.getAlliance().get().equals(Alliance.Red)){
      if(face == 0 || face == 1){
        id = 10;
      }
      else if(face == 2){
        id = 9;
      }
      else if(face == 3){
        id = 8;
      }
      else if(face == 4){
        id = 7;
      }
      else if(face == 5){
        id = 6;
      }
      else if(face == 6){
        id = 11;
      }
      else if(face == 10){
        id = 2;
      }
      else{
        id = 21;
      }
    }

    if(DriverStation.getAlliance().get().equals(Alliance.Blue)){
      if(face == 0 || face == 1){
        id = 21;
      }
      else if(face == 2){
        id = 22;
      }
      else if(face == 3){
        id = 17;
      }
      else if(face == 4){
        id = 18;
      }
      else if(face == 5){
        id = 19;
      }
      else if(face == 6){
        id = 20;
      }
      else{
        id = 21; //worst case should not occur
      }
    }


    // targetPose = new Pose2d(13.8, 5.66, Rotation2d.fromDegrees(60));
    targetPose = CommandSwerveDrivetrain.tagPoseAndymarkMap.get(id).transformBy(VisionConstants.rightBranch);
    System.out.println(targetPose);

    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        3.0, 4.0,
        Units.degreesToRadians(540), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    pathfindingCommand = AutoBuilder.pathfindToPose(
            targetPose,
            constraints,
            0.0 // Goal end velocity in meters/sec
    );
    pathfindingCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pathfindingCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathfindingCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathfindingCommand.isFinished();
  }
}
