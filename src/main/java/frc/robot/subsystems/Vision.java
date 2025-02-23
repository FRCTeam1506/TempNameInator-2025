// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  public static double tagX;// = LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_BACK)[3];
  public static double tagY;// = LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_BACK)[4];

  public static double align3d_x;
  public static double align3d_y;

  public Vision() {}

  @Override
  public void periodic() {
    if(LimelightHelpers.getTV(VisionConstants.LL_BACK)){

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("WPI_BPB (x)", LimelightHelpers.getCameraPose3d_TargetSpace(VisionConstants.LL_BACK).getX());
    SmartDashboard.putNumber("WPI_BPB (y)", LimelightHelpers.getCameraPose3d_TargetSpace(VisionConstants.LL_BACK).getY());
    SmartDashboard.putNumber("WPI_BPB (theta)", LimelightHelpers.getCameraPose3d_TargetSpace(VisionConstants.LL_BACK).getRotation().getAngle());

    SmartDashboard.putNumber("align3d_x", LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_BACK)[0]);
    SmartDashboard.putNumber("align3d_y", LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_BACK)[2]);
    align3d_x = LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_BACK)[2];
    align3d_y = LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_BACK)[0];

    double x = LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_BACK)[3];
    double y = LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_BACK)[4];
    double z = LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_BACK)[5];
    SmartDashboard.putNumber("Bot (x)", x - (x%0.1));
    SmartDashboard.putNumber("Bot (y)", y - (y%0.1));
    SmartDashboard.putNumber("Bot (z)", z - (z%0.1));

    tagX = x;
    tagY = y;

    double tpRS_x = LimelightHelpers.getTargetPose_RobotSpace(VisionConstants.LL_BACK)[3];
    double tpRS_y = LimelightHelpers.getTargetPose_RobotSpace(VisionConstants.LL_BACK)[4];
    double tpRS_z = LimelightHelpers.getTargetPose_RobotSpace(VisionConstants.LL_BACK)[5];
    SmartDashboard.putNumber("TPRS (x)", tpRS_x - (tpRS_x%0.1));
    SmartDashboard.putNumber("TPRS (y)", tpRS_y - (tpRS_y%0.1));
    SmartDashboard.putNumber("TPRS (z)", tpRS_z - (tpRS_z%0.1));


    }

  }
}
