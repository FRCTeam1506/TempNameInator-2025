// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.VisionConstants;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private final boolean kUseLimelight = true;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit(){
    PathfindingCommand.warmupCommand().schedule();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    /*
     * This example of adding Limelight is very simple and may not be sufficient for on-field use.
     * Users typically need to provide a standard deviation that scales with the distance to target
     * and changes with number of tags available.
     *
     * This example is sufficient to show that vision integration is possible, though exact implementation
     * of how to use vision should be tuned per-robot and to the team's specification.
     */


    if (kUseLimelight) {
      var driveState = m_robotContainer.drivetrain.getState();
      double headingDeg = driveState.Pose.getRotation().getDegrees(); //+180
      double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);
      m_robotContainer.drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.05, 0.05, 999999));

      LimelightHelpers.SetRobotOrientation(VisionConstants.LL_CENTER, headingDeg, 0, 0, 0, 0, 0);
      var llMeasurement1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.LL_CENTER);
      var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LL_CENTER);
      if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {

        Pose2d pose = new Pose2d(llMeasurement.pose.getX(), llMeasurement.pose.getY(), llMeasurement.pose.getRotation().minus(new Rotation2d(0))); //minus rotation2d(math.pi)
        m_robotContainer.drivetrain.addVisionMeasurement(pose, llMeasurement.timestampSeconds);

        SmartDashboard.putNumberArray("MT2Result", new double[]{llMeasurement.pose.getX(), llMeasurement.pose.getY()});
        // System.out.println("Updating MegaTag2!!!");
      }
    }

  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {

    if(DriverStation.getAlliance().get().equals(Alliance.Blue)){
      m_robotContainer.drivetrain.setOperatorPerspectiveForward(new Rotation2d(0));
    }
    else{
      m_robotContainer.drivetrain.setOperatorPerspectiveForward(new Rotation2d(Math.PI));
    }

  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.elevator.elevatorStop();
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("DTPose_X", m_robotContainer.drivetrain.getState().Pose.getX());
    SmartDashboard.putNumber("DTPose_Y", m_robotContainer.drivetrain.getState().Pose.getY());
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
