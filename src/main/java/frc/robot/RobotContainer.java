// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandPS4Controller driver = new CommandPS4Controller(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Climber climber = new Climber();
    public final Elevator elevator = new Elevator();
    public final Algae algae = new Algae();
    public final Coral coral = new Coral();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        j.dA.whileTrue(drivetrain.applyRequest(() -> brake));
        j.dB.whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));

        driver.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driver.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        j.dShare.and(j.dY).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        j.dShare.and(j.dX).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        j.dOptions.and(j.dY).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        j.dOptions.and(j.dX).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        j.dB.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));


        //ROBOT-SPECIFIC COMMANDS

        //climber commands
        j.oRB.whileTrue(new InstantCommand( () -> climber.up()));
        j.oLB.whileTrue(new InstantCommand( () -> climber.down()));
        j.oRB.whileFalse(new InstantCommand(() -> climber.stop()));
        j.oLB.whileFalse(new InstantCommand(() -> climber.stop()));

        //manual elevator commands
        j.oB.whileTrue(new InstantCommand( () -> elevator.elevatorDown()));
        j.oA.whileTrue(new InstantCommand( () -> elevator.elevatorUp()));
        j.oB.whileFalse(new InstantCommand(() -> elevator.elevatorStop()));
        j.oA.whileFalse(new InstantCommand(() -> elevator.elevatorStop()));

        //elevator setpoints
        // j.oUp.whileTrue(new InstantCommand(() -> elevator.elevatorL4()));
        // j.oRight.whileTrue(new InstantCommand(() -> elevator.elevatorL3()));
        // j.oLeft.whileTrue(new InstantCommand(() -> elevator.elevatorL2()));
        // j.oDown.whileTrue(new InstantCommand(() -> elevator.elevatorL1()));

        //algae gripper
        j.oY.whileTrue(new InstantCommand( () -> algae.intake()));
        j.oX.whileTrue(new InstantCommand( () -> algae.outtake()));
        j.oY.whileFalse(new InstantCommand(() -> algae.stop()));
        j.oX.whileFalse(new InstantCommand(() -> algae.stop()));

        j.oRT.onTrue(new InstantCommand(() -> algae.gripperUp())).onFalse(new InstantCommand(() -> algae.stop()));
        j.oLT.onTrue(new InstantCommand(() -> algae.gripperDown())).onFalse(new InstantCommand(() -> algae.stop()));

        j.oR3.whileTrue(new InstantCommand( () -> coral.intake()));
        j.oL3.whileTrue(new InstantCommand( () -> coral.reverse()));
        j.oR3.whileFalse(new InstantCommand(() -> coral.stop()));
        j.oL3.whileFalse(new InstantCommand(() -> coral.stop()));
    

        // if (j.oLT.equals(1)) {
        //     new InstantCommand(() -> algae.gripperLower());
        // }
        // if (j.oRT.equals(1)) {
        //     new InstantCommand(() -> algae.gripperRaise());
        // }
        


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() { 
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
