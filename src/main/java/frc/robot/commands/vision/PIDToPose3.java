// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDToPose3 extends Command {
  /** Creates a new PIDToPose. */
  CommandSwerveDrivetrain drivetrain;
  Pose2d goalPose;
  Pose2d currentFieldPose, targetPose;
  int id;
  int flip_for_red = 1;

  SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final ProfiledPIDController xController =
      new ProfiledPIDController(
          SwerveConstants.driveKP * 12,
          SwerveConstants.driveKI,
          SwerveConstants.driveKD * 4,
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

  Pose2d goalTagPose2d;



  public PIDToPose3(CommandSwerveDrivetrain drivetrain, Pose2d goalPose, int id) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.goalPose = goalPose;
    this.id = id;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentFieldPose = drivetrain.getState().Pose;

    Pose2d currentFieldPose2d = drivetrain.getState().Pose;
    Pose2d currentTagPose2d = currentFieldPose2d.relativeTo(CommandSwerveDrivetrain.tagPoseAndymarkMap.get(id));
    goalTagPose2d = goalPose.relativeTo(CommandSwerveDrivetrain.tagPoseAndymarkMap.get(id));


    xController.reset(currentTagPose2d.getX());
    yController.reset(currentTagPose2d.getY());
    thetaController.reset(currentTagPose2d.getRotation().getRadians());

    // xController.setGoal(goalTagPose2d.getX());
    // yController.setGoal(goalTagPose2d.getY());
    // thetaController.setGoal(goalTagPose2d.getRotation().getRadians());
    xController.setGoal(0);
    yController.setGoal(0);
    thetaController.setGoal(0);

    targetPose = CommandSwerveDrivetrain.tagPoseAndymarkMap.get(id);

    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
        flip_for_red = -1;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentFieldPose2d = drivetrain.getState().Pose;
    // Pose2d currentTagPose2d = currentFieldPose2d.relativeTo(targetPose);
    // Transform2d currentTagPose2d = currentFieldPose2d.minus(targetPose);
    Transform2d currentTagPose2d = targetPose.minus(currentFieldPose2d);

    goalTagPose2d = goalPose.relativeTo(CommandSwerveDrivetrain.tagPoseAndymarkMap.get(id));

    Translation2d fieldVelocity = new Translation2d(xController.calculate(currentTagPose2d.getX(),0), yController.calculate(currentTagPose2d.getY(),0));//.rotateBy(targetPose.getRotation());

    request
        .withVelocityX(fieldVelocity.getX() * flip_for_red)
        .withVelocityY(fieldVelocity.getY() * flip_for_red)
        .withRotationalRate(thetaController.calculate(currentTagPose2d.getRotation().getRadians()))
        .withDeadband(0.05)
        .withRotationalDeadband(Math.toRadians(2));

    // pathPIDRequest2.withSpeeds(new ChassisSpeeds(fieldVelocity.getX() * flip_for_red, fieldVelocity.getY() * flip_for_red, pathPIDRotationController.calculate(currentTagPose2d.getRotation().getRadians())));

    drivetrain.setControl(request);
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
