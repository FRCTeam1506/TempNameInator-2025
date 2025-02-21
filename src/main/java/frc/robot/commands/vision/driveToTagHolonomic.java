// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import java.lang.constant.Constable;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class driveToTagHolonomic extends Command {
  /** Creates a new driveToTagHolonomic. */
  boolean isFinished = false;

  PIDController xController = new PIDController(4, 0, .1);
  ProfiledPIDController thetaController = new ProfiledPIDController(6, 0, 0.1, new Constraints(2, 1));
  HolonomicDriveController controller = new HolonomicDriveController(xController, xController, thetaController);
  
  CommandSwerveDrivetrain drivetrain;
  SwerveRequest.ApplyRobotSpeeds request;

  public driveToTagHolonomic(CommandSwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;

    request = new SwerveRequest.ApplyRobotSpeeds();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!LimelightHelpers.getTV(Constants.VisionConstants.LL_BACK)){
      isFinished = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d goal = LimelightHelpers.getCameraPose3d_TargetSpace(VisionConstants.LL_BACK).toPose2d();
    Trajectory.State goalState = new State(0, 0, 0, goal, 0);
    ChassisSpeeds speeds = controller.calculate(new Pose2d(0, 0, drivetrain.getRotation3d().toRotation2d()), goalState, new Rotation2d(Math.toDegrees(drivetrain.getRotation3d().toRotation2d().getDegrees() + goal.getRotation().getDegrees())));
    drivetrain.setControl(request.withSpeeds(speeds));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(request.withSpeeds(new ChassisSpeeds(0, 0, 0)));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;// || !LimelightHelpers.getTV(VisionConstants.LL_BACK);
  }
}
