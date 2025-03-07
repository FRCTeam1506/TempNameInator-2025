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
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;

import com.ctre.phoenix6.swerve.SwerveRequest;


public class Jalign extends Command {
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
          SwerveConstants.driveKP,
          SwerveConstants.driveKI,
          SwerveConstants.driveKD,
          new TrapezoidProfile.Constraints(SwerveConstants.dMaxVelocity, SwerveConstants.dMaxAccel),
          0.02);

  private final ProfiledPIDController thetaController = 
      new ProfiledPIDController(
        SwerveConstants.alignKP, 
        SwerveConstants.alignKI, 
        SwerveConstants.alignKD, 
        new TrapezoidProfile.Constraints(SwerveConstants.tMaxVelocity, SwerveConstants.tMaxAccel));

        double goalAngle;


  public Jalign(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    this.timer = new Timer();
    addRequirements(drivetrain);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    request = new SwerveRequest.ApplyRobotSpeeds();
  }


  @Override
  public void initialize() {
    // Pose2d currentPose = drivetrain.getState().Pose;
    Pose2d currentPose = new Pose2d(Vision.align3d_x, Vision.align3d_y, new Rotation2d(Math.toRadians(LimelightHelpers.getTX(VisionConstants.LL_CENTER))));
    
    int tagId = (int) LimelightHelpers.getFiducialID(VisionConstants.LL_CENTER);

    if(tagId == -1){
      tagId = 0;
    }

    goalAngle = Math.toRadians(Vision.angles[tagId]);

    xController.reset(0);
    yController.reset(0);

    thetaController.reset(drivetrain.getState().Pose.getRotation().getRadians());
    thetaController.setGoal(goalAngle);
    thetaController.setTolerance(Math.toRadians(1.5));

    // thetaController.reset(currentPose.getRotation().getRadians());
    this.targetPose = new Pose2d(-0.3, 0.1, new Rotation2d(goalAngle));

    this.timer.restart();
  }

  @Override
  public void execute() {
    // set running to true in this method to capture that the calculate method has been invoked on
    // the PID controllers. This is important since these controllers will return true for atGoal if
    // the calculate method has not yet been invoked.
    running = true;


    // Pose2d currentPose = drivetrain.getState().Pose;
    // Pose2d currentPose = new Pose2d(0, 0, drivetrain.getRotation3d().toRotation2d());
    Pose2d currentPose = new Pose2d(Vision.align3d_x, Vision.align3d_y, drivetrain.getState().Pose.getRotation()); //new Rotation2d(Math.toRadians(LimelightHelpers.getTX(VisionConstants.LL_CENTER)))

    // use last values of filter
    double xVelocity = xController.calculate(currentPose.getX(), this.targetPose.getX());
    double yVelocity = yController.calculate(currentPose.getY(), this.targetPose.getY());

    // double theta = LimelightHelpers.getTX(VisionConstants.LL_CENTER); 
    double currentAngle = currentPose.getRotation().getDegrees();

    if(currentAngle > 360){
      do{
        currentAngle -= 360;
      }
      while(currentAngle>360);
    }
    else if(currentAngle < 0){
      do{
        currentAngle +=360;
      }
      while(currentAngle < 0); 
    }
    double rotationOutput = thetaController.calculate(Math.toRadians(currentAngle), goalAngle);   


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

    return this.timer.hasElapsed(timeout) || atGoal || !LimelightHelpers.getTV(VisionConstants.LL_CENTER);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(request.withSpeeds(new ChassisSpeeds(0, 0, 0)));
    running = false;
  }
}