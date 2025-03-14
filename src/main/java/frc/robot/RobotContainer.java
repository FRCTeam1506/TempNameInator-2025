// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.vision.DTPLeft;
import frc.robot.commands.vision.DTPoseTest;
import frc.robot.commands.vision.DriveToPose;
import frc.robot.commands.vision.DriveToPoseBeta;
import frc.robot.commands.vision.DriveToPoseBetaAutonomous;
import frc.robot.commands.vision.Jalign;
import frc.robot.commands.vision.OnlyTurn;
import frc.robot.commands.vision.OnlyTurnAprilTag;
import frc.robot.commands.vision.DriveToPoseBetaAutoNO;
import frc.robot.commands.vision.StopDrivetrain;
import frc.robot.commands.vision.align3d;
import frc.robot.commands.vision.align3dproper;
import frc.robot.commands.vision.alignRotationOnly;
import frc.robot.commands.vision.driveToTagHolonomic;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;
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
    public final Intake intake = new Intake();
    public final Vision vision = new Vision();
    public final Candle candle = new Candle();

    Autos autos = new Autos(drivetrain, algae, coral, elevator, intake);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;
    private SendableChooser<Command> autoChooserManual;

    public RobotContainer() {

        autos.makeNamedCommands();

        autoChooserManual = new SendableChooser<Command>();
        autoChooserManual = autos.configureChooser(autoChooserManual);

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        // SmartDashboard.putData("Auto Mode 2000", autoChooserManual);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed * (j.dLT.getAsBoolean() ? 0.25 : 1.0)) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed * (j.dLT.getAsBoolean() ? 0.25 : 1.0)) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate * (j.dLT.getAsBoolean() ? 0.5 : 1.0)) // Drive counterclockwise with negative X (left)
            )
        );


        j.dX.whileTrue(drivetrain.applyRequest(() -> brake));
        // j.dB.whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        // ));

        // driver.pov(0).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );
        // driver.pov(180).whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        // );

        // Run SysId routines when holding 
        //ack/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        j.dShare.and(j.dA).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        j.dShare.and(j.dX).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        j.dOptions.and(j.dA).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        j.dOptions.and(j.dX).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        j.dB.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));


        //ROBOT-SPECIFIC COMMANDS

        //climber commands
        j.dA.whileTrue(new InstantCommand( () -> climber.climb()));
        j.dY.whileTrue(new InstantCommand( () -> climber.unclimb()));
        j.dA.whileFalse(new InstantCommand(() -> climber.stop()));
        j.dY.whileFalse(new InstantCommand(() -> climber.stop()));

        //manual elevator commands -- a up, b down
        j.oOptions.whileTrue(new InstantCommand( () -> elevator.elevatorDown()));
        j.oTouchpad.whileTrue(new InstantCommand( () -> elevator.elevatorUp()));
        j.oOptions.whileFalse(new InstantCommand(() -> elevator.elevatorStop()));
        j.oTouchpad.whileFalse(new InstantCommand(() -> elevator.elevatorStop()));

        //elevator setpoints
        j.oUp.whileTrue(new InstantCommand(() -> elevator.elevatorL4()));
        j.oRight.whileTrue(new InstantCommand(() -> elevator.elevatorL3()));
        j.oLeft.whileTrue(new InstantCommand(() -> elevator.elevatorL2()));
        j.oDown.whileTrue(new InstantCommand(() -> elevator.elevatorGround()));
        // j.oUp.whileFalse(new InstantCommand(() -> elevator.elevatorStop()));
        // j.oRight.whileFalse(new InstantCommand(() -> elevator.elevatorStop()));
        // j.oLeft.whileFalse(new InstantCommand(() -> elevator.elevatorStop()));
        // j.oDown.whileFalse(new InstantCommand(() -> elevator.elevatorStop())); //uncomment if we want to hold down


        //algae gripper
        j.oY.whileTrue(new InstantCommand( () -> algae.intake()));
        j.oX.whileTrue(new InstantCommand( () -> algae.outtake()));
        j.oY.whileFalse(new InstantCommand(() -> algae.stop()));
        j.oX.whileFalse(new InstantCommand(() -> algae.stop()));

        // j.dA.whileTrue(new InstantCommand(() -> algae.outtake()));
        // j.dA.whileFalse(new InstantCommand(() -> algae.stopIntake()));

        // j.oR3.onTrue(new InstantCommand(() -> algae.gripperUp())).onFalse(new InstantCommand(() -> algae.stopVertical()));
        // j.oL3.onTrue(new InstantCommand(() -> algae.gripperDown())).onFalse(new InstantCommand(() -> algae.stopVertical()));

        Trigger oRTNew = new Trigger(() -> j.operator.getRawAxis(4) > 0.1);

        j.oLT.whileTrue(new InstantCommand(() -> algae.verticalScore()).alongWith(new InstantCommand(() -> algae.intake()))).whileFalse(new InstantCommand(() -> algae.verticalHome()).andThen(new InstantCommand(() -> algae.stopIntake())));
        j.oRT.whileTrue(new InstantCommand(() -> algae.verticalBarge())).onFalse(new InstantCommand(() -> algae.outtake()));
        // oRTNew.whileTrue(new RepeatCommand(new InstantCommand(() -> algae.gripperUp(-j.operator.getRawAxis(4) * 0.2))));
        // j.oLT.whileFalse(new InstantCommand(() -> algae.stop()));
        j.oRT.whileFalse(new InstantCommand(() -> algae.stop()));

        j.oL3.whileTrue(new InstantCommand(() -> algae.gripperUp()));
        j.oR3.whileTrue(new InstantCommand(() -> algae.gripperDown()));
        j.oL3.whileFalse(new InstantCommand(() -> algae.stopVertical()));
        j.oR3.whileFalse(new InstantCommand(() -> algae.stopVertical()));

        //normal coral intake
        j.oA.whileTrue(new InstantCommand(() -> coral.switchIntake()));
        j.oB.whileTrue(new InstantCommand( () -> coral.switchOuttake()));
        j.dRT.whileTrue(new InstantCommand(() -> coral.switchIntake()));
        j.oA.whileFalse(new InstantCommand(() -> coral.stop()));
        j.oB.whileFalse(new InstantCommand(() -> coral.stop()));
        j.dRT.whileFalse(new InstantCommand(() -> coral.stop()));
        
        //floor intake
        j.dUp.whileTrue(new InstantCommand(() -> intake.up()));
        j.dDown.whileTrue(new InstantCommand(() -> intake.down()));
        j.dUp.whileFalse(new InstantCommand(() -> intake.stop()));
        j.dDown.whileFalse(new InstantCommand(() -> intake.stop()));

        //zeroing things --- driver for side intake, operator for algae gripper
        j.dPS.whileTrue(new InstantCommand(() -> intake.zeroVertical()));
        j.oPS.whileTrue(new InstantCommand(() -> algae.zeroVertical()));

        //operator floor intake macros
        j.oRB.whileTrue(new InstantCommand(() -> intake.lowerIntake()));
        j.oRB.whileTrue(new InstantCommand(() -> intake.intake()));
        j.oRB.whileFalse(new InstantCommand(() -> intake.intakeIdle()));
        j.oRB.whileFalse(new InstantCommand(() -> intake.raiseIntake()));

        j.oLB.whileTrue(new InstantCommand(() -> intake.scoreIntake()));
        j.oLB.whileTrue(new InstantCommand(() -> intake.outtake()));
        j.oLB.whileFalse(new InstantCommand(() -> intake.stopIntake()));
        j.oLB.whileFalse(new InstantCommand(() -> intake.raiseIntake()));

        //alignment to apriltag
        j.dLB.whileTrue(new DTPLeft(drivetrain));
        j.dRB.whileTrue(new DriveToPoseBeta(drivetrain));
        j.dLeft.whileTrue(new Jalign(drivetrain));
        j.dRight.whileTrue(new OnlyTurnAprilTag(drivetrain));

        j.dX.whileTrue(new InstantCommand(() -> candle.toggleNoah()));


        // j.dOptions.onTrue(new InstantCommand(() -> test.runTest(10))); //10 Is always start.
        // j.dOptions.onFalse(new InstantCommand(() -> test.runTest(0))); //10 Is always start.


    

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() { 
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
