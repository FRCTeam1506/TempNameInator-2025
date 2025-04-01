// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDToPose extends Command {
  /** Creates a new PIDToPose. */
  CommandSwerveDrivetrain drivetrain;
  Pose2d goalPose;
  Pose2d currentFieldPose;

  SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final ProfiledPIDController xController =
      new ProfiledPIDController(
          SwerveConstants.driveKP *1.5,
          SwerveConstants.driveKI,
          SwerveConstants.driveKD,
          new TrapezoidProfile.Constraints(SwerveConstants.dMaxVelocity * 1.8, SwerveConstants.dMaxAccel),
          0.02);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(
          SwerveConstants.driveKP * 1.5,
          SwerveConstants.driveKI,
          SwerveConstants.driveKD,
          new TrapezoidProfile.Constraints(SwerveConstants.dMaxVelocity * 1.8, SwerveConstants.dMaxAccel),
          0.02);
  ProfiledPIDController thetaController = 
      new ProfiledPIDController(
        SwerveConstants.alignKP,
        SwerveConstants.alignKI,
        SwerveConstants.alignKD,
        new TrapezoidProfile.Constraints(SwerveConstants.tMaxVelocity, SwerveConstants.tMaxAccel),
        0.02);


  public PIDToPose(CommandSwerveDrivetrain drivetrain, Pose2d goalPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.goalPose = goalPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentFieldPose = drivetrain.getState().Pose;

    Pose2d diff = currentFieldPose.relativeTo(goalPose);
    xController.reset(diff.getX());
    yController.reset(diff.getY());
    thetaController.reset(diff.getRotation().getRadians());

    xController.setGoal(goalPose.getX());
    yController.setGoal(goalPose.getY());
    thetaController.setGoal(goalPose.getRotation().getRadians());

    xController.setTolerance(0.05);
    yController.setTolerance(0.05);
    thetaController.setTolerance(Math.toRadians(3));

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentFieldPose = drivetrain.getState().Pose;

    double xSpeed = xController.calculate(currentFieldPose.getX());
    double ySpeed = yController.calculate(currentFieldPose.getY());
    double rotSpeed = thetaController.calculate(currentFieldPose.getRotation().getRadians());

    drivetrain.setControl(request.withVelocityX(xSpeed).withVelocityY(-ySpeed).withRotationalRate(rotSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(request.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }
}
