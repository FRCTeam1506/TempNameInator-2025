package frc.robot.commands.vision;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

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
public class PoseAlignAuto extends Command {
  private final CommandSwerveDrivetrain drivetrain;

  SwerveRequest.ApplyRobotSpeeds request;
  boolean isFinished = false;

  private boolean running = false;
  private Timer timer;
  double closeVelocityBoost = 0.0;
  double timeout = 0.7;

  private final PIDController xController =
      new PIDController(
          SwerveConstants.driveKP, // * 1.4?
          SwerveConstants.driveKI,
          SwerveConstants.driveKD * 1.25); // * 1.25? * 1?
  private final PIDController yController =
      new PIDController(
          SwerveConstants.driveKP,
          SwerveConstants.driveKI,
          SwerveConstants.driveKD * 1.25);

  ProfiledPIDController thetaController = new ProfiledPIDController(SwerveConstants.alignKP * 4, SwerveConstants.alignKI, SwerveConstants.alignKD, new Constraints(SwerveConstants.tMaxVelocity, SwerveConstants.tMaxAccel));


  private HolonomicDriveController holonomicDriveController;
  private Pose2d startPos = new Pose2d();
  private Pose2d goalPose = new Pose2d();

  int thetaGoal;
  boolean left;

  /**
   * @param drivetrain the drivetrain subsystem required by this command
   * @param left true if aligning to left side, false if aligning to right side
   */
  public PoseAlignAuto(CommandSwerveDrivetrain drivetrain, boolean left) {
    this.drivetrain = drivetrain;
    this.timer = new Timer();
    addRequirements(drivetrain);

    request = new SwerveRequest.ApplyRobotSpeeds();
    this.left = left;
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

    isFinished = false;

    holonomicDriveController =
        new HolonomicDriveController(xController, yController, thetaController);
    holonomicDriveController.setTolerance(new Pose2d(0.02, 0.02, Rotation2d.fromDegrees(1.5)));

    startPos = drivetrain.getState().Pose;

    int id = (int) drivetrain.getTag();
    if(id == -1){
      id = 6;
      isFinished = true;
    }
    if(left){
      goalPose = CommandSwerveDrivetrain.tagPoseAndymarkMap.get(id).transformBy(VisionConstants.leftBranch);
    }
    else{
      goalPose = CommandSwerveDrivetrain.tagPoseAndymarkMap.get(id).transformBy(VisionConstants.rightBranch);
    }

    this.timer.restart();
    System.out.println("Goal(x): " + goalPose.getX() + "\nGoal(y): " + goalPose.getY() + "\nGoal(rot): " + goalPose.getRotation().getDegrees());
  }

  /**
   * This method is invoked periodically while this command is scheduled. It calculates the
   * velocities based on the current and target poses and invokes the drivetrain subsystem's drive
   * method.
   */
  @Override
  public void execute() {
    running = true;

    Pose2d currPose2d = drivetrain.getState().Pose;
    ChassisSpeeds chassisSpeeds = this.holonomicDriveController.calculate(currPose2d, goalPose, 0, goalPose.getRotation());

    drivetrain.setControl(request.withSpeeds(chassisSpeeds));
  }

  @Override
  public boolean isFinished() {
    return this.timer.hasElapsed(timeout) || holonomicDriveController.atReference() || isFinished;
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(request.withSpeeds(new ChassisSpeeds(0, 0, 0)));
    running = false;
  }
}