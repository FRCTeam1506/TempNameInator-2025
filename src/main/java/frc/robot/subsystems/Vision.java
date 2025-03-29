// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  private CommandSwerveDrivetrain drivetrain;

  public static double tagX;// = LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_BACK)[3];
  public static double tagY;// = LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_BACK)[4];

  public static double align3d_x;
  public static double align3d_y;

  public static double align3d_x_left;
  public static double align3d_y_left;

  public static int[] angles;

  public SwerveDrivePoseEstimator estimator;

  public static Map<Integer, Pose2d> tagPoseAndymarkMap;



  public Vision(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    angles = new int[23];

    for(int i = 0; i<angles.length; i++){
      angles[i] = 0;
    }

    angles[21] = 180;
    angles[10] = 180;
    angles[20] = 60;
    angles[11] = -120;
    angles[19] = -60;
    angles[6] = -60;

    angles[18] = 0;
    angles[7] = 0;
    angles[17] = -120;
    angles[8] = 60;
    angles[22] = 120; //new
    angles[9] = 120;

    // angles[21] = 0;
    // angles[10] = 0;
    // angles[20] = 60;
    // angles[11] = 60;
    // angles[19] = 120;
    // angles[6] = 120;

    // angles[18] = 180;
    // angles[7] = 180;
    // angles[17] = -120;
    // angles[8] = -120;
    // angles[22] = -60;
    // angles[9] = -60;
  }

  public void initializePoseEstimator(Pose2d pose){

    Translation2d[] swerveModuleLocations = new Translation2d[4];
    swerveModuleLocations[0] = new Translation2d(TunerConstants.kFrontLeftXPos, TunerConstants.kFrontLeftYPos);
    swerveModuleLocations[1] = new Translation2d(TunerConstants.kFrontRightXPos, TunerConstants.kFrontRightYPos);
    swerveModuleLocations[2] = new Translation2d(TunerConstants.kBackLeftXPos, TunerConstants.kBackLeftYPos);
    swerveModuleLocations[3] = new Translation2d(TunerConstants.kFrontRightXPos, TunerConstants.kFrontRightYPos);

    SwerveDriveKinematics swerve_kinematics = new SwerveDriveKinematics(swerveModuleLocations);



    estimator = new SwerveDrivePoseEstimator(swerve_kinematics, drivetrain.getState().Pose.getRotation(), drivetrain.getState().ModulePositions, pose);
  }

  public void initializeAndyMarkMap(){
    tagPoseAndymarkMap.put(6, new Pose2d(13.474, 3.301, new Rotation2d(Math.toRadians(-60))));
    tagPoseAndymarkMap.put(7, new Pose2d(13.890, 4.021, new Rotation2d(Math.toRadians(0))));
    tagPoseAndymarkMap.put(8, new Pose2d(13.474, 4.740, new Rotation2d(Math.toRadians(60))));
    tagPoseAndymarkMap.put(9, new Pose2d(12.643, 4.740, new Rotation2d(Math.toRadians(120))));
    tagPoseAndymarkMap.put(10, new Pose2d(12.227, 4.021, new Rotation2d(Math.toRadians(180))));
    tagPoseAndymarkMap.put(11, new Pose2d(12.643, 3.301, new Rotation2d(Math.toRadians(-120))));

    tagPoseAndymarkMap.put(17, new Pose2d(4.074, 3.301, new Rotation2d(Math.toRadians(-120))));
    tagPoseAndymarkMap.put(18, new Pose2d(3.658, 4.021, new Rotation2d(Math.toRadians(180))));
    tagPoseAndymarkMap.put(19, new Pose2d(4.074, 4.740, new Rotation2d(Math.toRadians(120))));
    tagPoseAndymarkMap.put(20, new Pose2d(4.905, 4.740, new Rotation2d(Math.toRadians(60))));
    tagPoseAndymarkMap.put(21, new Pose2d(5.321, 4.021, new Rotation2d(Math.toRadians(0))));
    tagPoseAndymarkMap.put(22, new Pose2d(4.905, 3.301, new Rotation2d(Math.toRadians(-60))));
  }

  @Override
  public void periodic() {
    if(LimelightHelpers.getTV(VisionConstants.LL_CENTER)){

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("WPI_BPB (x)", LimelightHelpers.getCameraPose3d_TargetSpace(VisionConstants.LL_CENTER).getX());
    SmartDashboard.putNumber("WPI_BPB (y)", LimelightHelpers.getCameraPose3d_TargetSpace(VisionConstants.LL_CENTER).getY());
    SmartDashboard.putNumber("WPI_BPB (theta)", LimelightHelpers.getCameraPose3d_TargetSpace(VisionConstants.LL_CENTER).getRotation().getAngle());

    SmartDashboard.putNumber("align3d_x", LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_CENTER)[2]);
    SmartDashboard.putNumber("align3d_y", LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_CENTER)[0]);

    align3d_x = LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_CENTER)[2];
    align3d_y = LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_CENTER)[0];


    double x = LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_CENTER)[3];
    double y = LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_CENTER)[4];
    double z = LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_CENTER)[5];
    SmartDashboard.putNumber("Bot (x)", x - (x%0.1));
    SmartDashboard.putNumber("Bot (y)", y - (y%0.1));
    SmartDashboard.putNumber("Bot (z)", z - (z%0.1));

    tagX = x;
    tagY = y;

    double tpRS_x = LimelightHelpers.getTargetPose_RobotSpace(VisionConstants.LL_CENTER)[3];
    double tpRS_y = LimelightHelpers.getTargetPose_RobotSpace(VisionConstants.LL_CENTER)[4];
    double tpRS_z = LimelightHelpers.getTargetPose_RobotSpace(VisionConstants.LL_CENTER)[5];
    SmartDashboard.putNumber("TPRS (x)", tpRS_x - (tpRS_x%0.1));
    SmartDashboard.putNumber("TPRS (y)", tpRS_y - (tpRS_y%0.1));
    SmartDashboard.putNumber("TPRS (z)", tpRS_z - (tpRS_z%0.1));


    }

    if(LimelightHelpers.getTV(VisionConstants.LL_LEFT)){

      // SmartDashboard.putNumber("align3d_x_left", LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_LEFT)[2]);
      // SmartDashboard.putNumber("align3d_y_left", LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_LEFT)[0]);  

      // align3d_x_left = LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_LEFT)[2];
      // align3d_y_left = LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_LEFT)[0];
  
    }
    else{
      // System.out.println("Left: " + LimelightHelpers.getTV(VisionConstants.LL_LEFT));
    }


    try {
      SmartDashboard.putNumber("align3d_x_left", LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_LEFT)[2]);
      SmartDashboard.putNumber("align3d_y_left", LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_LEFT)[0]);  

      align3d_x_left = LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_LEFT)[2];
      align3d_y_left = LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_LEFT)[0];


    } catch (Exception e) {
      // TODO: handle exception
      System.out.println("Vision periodic failed");

    }

  }
}
