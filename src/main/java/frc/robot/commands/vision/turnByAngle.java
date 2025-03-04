// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class turnByAngle extends Command {
  /** Creates a new turnToAngle. */
  private final CommandSwerveDrivetrain drivetrain;
  double angle, initialAngle, targetGyro;

  private final ProfiledPIDController aimController;

    //From tunerconsts and robtocontainer.java
    private static final double MAX_AIM_VELOCITY = Math.PI; // radd/s
    private static final double MAX_AIM_ACCELERATION = Math.PI / 2; // rad/s^2
  
    // Todo - Tune later
    private static final double AIM_P = 4; //Proprotinal
    private static final double AIM_I = 0.01; //Gradual corretction
    private static final double AIM_D = 0.3;//0.05; //Smooth oscilattions

    SwerveRequest.FieldCentricFacingAngle request;
    SwerveRequest.ApplyRobotSpeeds stopRequest;
    SwerveRequest.RobotCentric alignRequest;

  /**
   * @param angle Angle you want to robot to turn compared to its current position (degrees).
   */

  public turnByAngle(CommandSwerveDrivetrain drivetrain, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.angle = angle;

    request = new SwerveRequest.FieldCentricFacingAngle().withDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1).withRotationalDeadband(0.1);
    stopRequest = new SwerveRequest.ApplyRobotSpeeds();
    alignRequest = new SwerveRequest.RobotCentric().withDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1).withRotationalDeadband(0.01);

    aimController = new ProfiledPIDController(AIM_P, AIM_I, AIM_D, new TrapezoidProfile.Constraints(MAX_AIM_VELOCITY, MAX_AIM_ACCELERATION));
    aimController.enableContinuousInput(-Math.PI, Math.PI); //Wrpa from -pi to ip
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialAngle = drivetrain.getPigeon2().getYaw().getValueAsDouble();
    targetGyro = initialAngle - angle;

    aimController.setGoal(Math.toRadians(targetGyro));

    System.out.println("Initializing!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotationOutput = aimController.calculate(Math.toRadians(drivetrain.getPigeon2().getYaw().getValueAsDouble()));
    drivetrain.setControl(alignRequest.withRotationalRate(rotationOutput));

    // drivetrain.setControl(request.withTargetRateFeedforward(rotationOutput));//.withVelocityX(0).withVelocityY(0));
    // if(drivetrain.getPigeon2().getRotation2d().getDegrees() >=85 || drivetrain.getPigeon2().getRotation2d().getDegrees() <= 95) {
    //     drivetrain.setControl(stopRequest.withSpeeds(new ChassisSpeeds(0, 0, 0)));
    // }

    // drivetrain.setControl(request.withTargetDirection(new Rotation2d(Math.toRadians(angle))).withVelocityX(0).withVelocityY(1));
    System.out.println("Running!");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(stopRequest.withSpeeds(new ChassisSpeeds(0, 0, 0)));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
