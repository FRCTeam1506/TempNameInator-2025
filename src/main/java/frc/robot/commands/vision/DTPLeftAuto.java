//derived from drivetoposebeta

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
public class DTPLeftAuto extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private Pose2d targetPose;

  SwerveRequest.ApplyRobotSpeeds request;

  private boolean running = false;
  private Timer timer;
  double closeVelocityBoost = 0.0;
  double timeout = 10;

  private final ProfiledPIDController xController =
      new ProfiledPIDController(
          SwerveConstants.driveKP,
          SwerveConstants.driveKI,
          SwerveConstants.driveKD,
          new TrapezoidProfile.Constraints(SwerveConstants.dMaxVelocity, SwerveConstants.dMaxAccel),
          0.02);
  private final ProfiledPIDController yController =
      new ProfiledPIDController(
          SwerveConstants.driveKP * 1.5,
          SwerveConstants.driveKI,
          SwerveConstants.driveKD * 5,
          new TrapezoidProfile.Constraints(SwerveConstants.dMaxVelocity, SwerveConstants.dMaxAccel),
          0.02);

  private final ProfiledPIDController thetaController = 
      new ProfiledPIDController(
        SwerveConstants.alignKP, 
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
  public DTPLeftAuto(CommandSwerveDrivetrain drivetrain) {
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
    this.targetPose = new Pose2d(-0.20, -0.04, new Rotation2d(Math.toRadians(19)));

    thetaController.setGoal(targetPose.getRotation().getRadians());
    thetaController.setTolerance(Math.toRadians(1.5));

    xController.reset(0);
    yController.reset(0);
    thetaController.reset(0);

    this.timer.restart();
  }

  /**
   * This method is invoked periodically while this command is scheduled. It calculates the
   * velocities based on the current and target poses and invokes the drivetrain subsystem's drive
   * method.
   */
  @Override
  public void execute() {
    running = true;

    Pose2d currentPose = new Pose2d(Vision.align3d_x_left, Vision.align3d_y_left, new Rotation2d(Math.toRadians(LimelightHelpers.getTX(VisionConstants.LL_LEFT))));

    // use last values of filter
    double xVelocity = xController.calculate(currentPose.getX(), this.targetPose.getX());
    double yVelocity = yController.calculate(currentPose.getY(), this.targetPose.getY());

    double theta = LimelightHelpers.getTX(VisionConstants.LL_LEFT);     
    // double theta = drivetrain.getPigeon2().getYaw().getValueAsDouble();
    double rotationOutput = thetaController.calculate(Math.toRadians(theta));
    // System.out.println("Theta: " + theta + "\nSpeedset: " + rotationOutput);

    //thetaVelocity add it back
    drivetrain.setControl(request.withSpeeds(new ChassisSpeeds(-xVelocity,yVelocity,rotationOutput)));
  }


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
    return this.timer.hasElapsed(timeout) || atGoal || !LimelightHelpers.getTV(VisionConstants.LL_LEFT);
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