// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// originally from https://github.com/Mechanical-Advantage/RobotCode2023

package frc.robot.commands.vision;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

//import frc.lib.team3061.RobotConfig;
//import frc.lib.team3061.drivetrain.Drivetrain;
//import frc.lib.team6328.util.LoggedTunableNumber;
//import frc.robot.Field2d;
import java.util.function.Supplier;
//import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;

/**
 * This command, when executed, instructs the drivetrain subsystem to drive to the specified pose in
 * a straight line. The execute method invokes the drivetrain subsystem's drive method. For
 * following a predetermined path, refer to the FollowPath Command class. For generating a path on
 * the fly and following that path, refer to the MoveToPose Command class.
 *
 * <p>Requires: the Drivetrain subsystem
 *
 * <p>Finished When: the robot is at the specified pose (within the specified tolerances)
 *
 * <p>At End: stops the drivetrain
 */
public class DTPoseTest extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private Pose2d targetPose, targetPoseNew;

  SwerveRequest.ApplyRobotSpeeds request;

  private boolean running = false;
  private Timer timer;
  double closeVelocityBoost = 0.0;
  double timeout = 2;

  private final ProfiledPIDController xController =
      new ProfiledPIDController(
          SwerveConstants.driveKP,
          SwerveConstants.driveKI,
          SwerveConstants.driveKD,
          new TrapezoidProfile.Constraints(SwerveConstants.dMaxVelocity, SwerveConstants.dMaxAccel),
          0.02);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(
          SwerveConstants.driveKP,
          SwerveConstants.driveKI,
          SwerveConstants.driveKD,
          new TrapezoidProfile.Constraints(SwerveConstants.dMaxVelocity, SwerveConstants.dMaxAccel),
          0.02);

  private final ProfiledPIDController thetaController = 
      new ProfiledPIDController(
        SwerveConstants.alignKP / 4, 
        SwerveConstants.alignKI, 
        SwerveConstants.alignKD, 
        new TrapezoidProfile.Constraints(SwerveConstants.tMaxVelocity, SwerveConstants.tMaxAccel));

  /**
   * Constructs a new DriveToPose command that drives the robot in a straight line to the specified
   * pose. A pose supplier is specified instead of a pose since the target pose may not be known
   * when this command is created.
   *
   * @param drivetrain the drivetrain subsystem required by this command
   * @param poseSupplier a supplier that returns the pose to drive to
   */
  public DTPoseTest(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    this.timer = new Timer();
    addRequirements(drivetrain);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    request = new SwerveRequest.ApplyRobotSpeeds();
  }

  /**
   * This method is invoked once when this command is scheduled. It resets all the PID controllers
   * and initializes the current and target poses. It is critical that this initialization occurs in
   * this method and not the constructor as this object is constructed well before the command is
   * scheduled and the robot's pose will definitely have changed and the target pose may not be
   * known until this command is scheduled.
   */
  @Override
  public void initialize() {
    // Reset all controllers
    Pose2d dtPose = drivetrain.getState().Pose;
    Pose2d currentPose;

    if(LimelightHelpers.getTV(VisionConstants.LL_LEFT)){
      currentPose = new Pose2d(Vision.align3d_x, Vision.align3d_y, new Rotation2d(Math.toRadians(LimelightHelpers.getTX(VisionConstants.LL_CENTER))));
    }
    else{
      currentPose = new Pose2d(0,0, new Rotation2d(0));
    }

    this.targetPose = new Pose2d(0, 0, new Rotation2d(Math.toRadians(31)));


    double newX = dtPose.getX() + currentPose.getX();
    double newY = dtPose.getY() - currentPose.getY();
    Rotation2d newTheta = targetPose.getRotation();

    targetPoseNew = new Pose2d(newX, newY, newTheta);
    // xController.reset(currentPose.getX());
    // yController.reset(currentPose.getY());

    xController.reset(currentPose.getX());
    yController.reset(currentPose.getY());

    xController.setGoal(targetPoseNew.getX());
    yController.setGoal(targetPoseNew.getY());

    thetaController.setGoal(targetPoseNew.getRotation().getRadians());

    thetaController.setTolerance(Math.toRadians(1.5));

    xController.setTolerance(0.02);
    yController.setTolerance(0.08);

    // thetaController.reset(currentPose.getRotation().getRadians());

    this.timer.restart();
  }

  @Override
  public void execute() {
    // set running to true in this method to capture that the calculate method has been invoked on
    // the PID controllers. This is important since these controllers will return true for atGoal if
    // the calculate method has not yet been invoked.
    running = true;

    if(LimelightHelpers.getTV(VisionConstants.LL_CENTER)){
      targetPoseNew = new Pose2d(Vision.align3d_x, Vision.align3d_y, new Rotation2d(Math.toRadians(0)));
    }


    Pose2d currentPose = drivetrain.getState().Pose;
    // Pose2d currentPose = new Pose2d(0, 0, drivetrain.getRotation3d().toRotation2d());
    // Pose2d currentPose = new Pose2d(Vision.align3d_x, Vision.align3d_y, new Rotation2d(Math.toRadians(LimelightHelpers.getTX(VisionConstants.LL_BACK))));

    // use last values of filter
    double xVelocity = xController.calculate(currentPose.getX(), this.targetPoseNew.getX());
    double yVelocity = yController.calculate(currentPose.getY(), this.targetPoseNew.getY());

    double theta = LimelightHelpers.getTX(VisionConstants.LL_CENTER);     
    double rotationOutput = thetaController.calculate(Math.toRadians(theta));   


    //thetaVelocity add it back
    drivetrain.setControl(request.withSpeeds(new ChassisSpeeds(xVelocity,yVelocity,0)));
  }

  /**
   * This method returns true if the command has finished. It is invoked periodically while this
   * command is scheduled (after execute is invoked). This command is considered finished if the
   * move-to-pose feature is disabled on the drivetrain subsystem or if the timeout has elapsed or
   * if all the PID controllers are at their goal.
   *
   * @return true if the command has finished
   */
  @Override
  public boolean isFinished() {
    Transform2d difference = drivetrain.getState().Pose.minus(targetPose);

    boolean atGoal =
        Math.abs(difference.getX()) < 0.01
            && Math.abs(difference.getY()) < 0.01
            && Math.abs(difference.getRotation().getRadians())
                < Math.toRadians(3);

    // check that running is true (i.e., the calculate method has been invoked on the PID
    // controllers) and that each of the controllers is at their goal. This is important since these
    // controllers will return true for atGoal if the calculate method has not yet been invoked.
    return this.timer.hasElapsed(timeout) || atGoal || (xController.atGoal() && yController.atGoal() && thetaController.atGoal());
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. It stops the motion
   * of the drivetrain.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(request.withSpeeds(new ChassisSpeeds(0, 0, 0)));
    running = false;
  }
}