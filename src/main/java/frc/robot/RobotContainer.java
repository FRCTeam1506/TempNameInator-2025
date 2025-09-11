// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//THIS IS FOR PS4 CONTROLLER FOR DRIVER!!!!!!! This is the origin
package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.autoScore;
import frc.robot.commands.controllers.Rumble1;
import frc.robot.commands.controllers.RumbleUntimed;
import frc.robot.commands.macros.AlgaeL2;
import frc.robot.commands.macros.AlgaeL3;
import frc.robot.commands.pathfinding.PathfindingCommand;
import frc.robot.commands.pathfinding.SmartPathfinding;
import frc.robot.commands.vision.DTPLeft;
import frc.robot.commands.vision.DTPoseTest;
import frc.robot.commands.vision.DriveToPose;
import frc.robot.commands.vision.DriveToPoseBeta;
import frc.robot.commands.vision.DriveToPoseBetaAutonomous;
import frc.robot.commands.vision.Jalign;
import frc.robot.commands.vision.JalignLeft;
import frc.robot.commands.vision.JalignRight;
import frc.robot.commands.vision.OnlyTurn;
import frc.robot.commands.vision.OnlyTurn2;
import frc.robot.commands.vision.OnlyTurn2Deg;
import frc.robot.commands.vision.OnlyTurnAprilTag;
import frc.robot.commands.vision.PIDToPose;
import frc.robot.commands.vision.PIDToPose2;
import frc.robot.commands.vision.PIDToPose3;
import frc.robot.commands.vision.PIDToPose4Spark;
import frc.robot.commands.vision.PIDToPose5Holonomic;
import frc.robot.commands.vision.PoseAlign;
import frc.robot.commands.vision.PoseAlignBargeAuto;
import frc.robot.commands.vision.PoseAlignHP;
import frc.robot.commands.vision.PoseAlignRight;
import frc.robot.commands.vision.PoseAlignToAutoStartingPt;
import frc.robot.commands.vision.DriveToPoseBetaAutoNO;
import frc.robot.commands.vision.StopDrivetrain;
import frc.robot.commands.vision.TTAHolonomicAprilTag;
import frc.robot.commands.vision.TurnToAngleHolonomic;
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

    boolean Mode;
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
    private final SwerveRequest.FieldCentric forwardStraightFieldCentric = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandPS4Controller driver = new CommandPS4Controller(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Climber climber = new Climber();
    public final Elevator elevator = new Elevator();
    // public final Algae algae = new Algae();
    public final Coral coral = new Coral();
    public final Intake intake = new Intake();
    public final Vision vision = new Vision();
    public final Candle candle = new Candle(drivetrain);
    public final Algae algae = new Algae();
    

    Autos autos = new Autos(drivetrain, algae, coral, elevator, intake, climber);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;
    private SendableChooser<Command> autoChooserManual;
    public static SendableChooser<String> lazyAuto2000 = new SendableChooser<String>();
    public static SendableChooser<Integer> autoDriveLocation = new SendableChooser<Integer>();


    public RobotContainer() {

        autos.makeNamedCommands();

        autoChooserManual = new SendableChooser<Command>();
        autoChooserManual = autos.configureChooser(autoChooserManual);
        lazyAuto2000.addOption("Left", "Left");
        lazyAuto2000.addOption("Center", "Center");
        lazyAuto2000.addOption("Right", "Right");

        autoDriveLocation.addOption("Center Rear (id 10/21)", 1);
        autoDriveLocation.addOption("Right Rear (id 9/22)", 2);
        autoDriveLocation.addOption("Right Front (id 8/17)", 3);
        autoDriveLocation.addOption("Center Front (id 7/18)", 4);
        autoDriveLocation.addOption("Left Front (id 6/19)", 5);
        autoDriveLocation.addOption("Left Rear (id 11/20)", 6);
        autoDriveLocation.addOption("Right HP (id 2/?)", 10);

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putData("lazyAuto2000", lazyAuto2000);
        SmartDashboard.putData("SmartPathfinding", autoDriveLocation);
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

        j.dOptions.whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(0.5))
        );
        j.dTouchpad.whileTrue(drivetrain.applyRequest(() ->
        forwardStraight.withVelocityX(0).withVelocityY(-0.5))
        );

        // j.dOptions.whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraightFieldCentric.withVelocityX(0.5).withVelocityY(0)));
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

        driver.setRumble(GenericHID.RumbleType.kBothRumble, 1.0);

        j.dA.and(j.dRT).whileTrue(new RepeatCommand(new InstantCommand(() -> climber.turboClimb())));
        j.dA.and(j.dRT).whileFalse(new InstantCommand(() -> climber.stop()));

        //manual elevator commands -- a up, b down
        j.oOptions.whileTrue(new InstantCommand( () -> elevator.elevatorDown()));
        j.oTouchpad.whileTrue(new InstantCommand( () -> elevator.elevatorUp()));
        j.oOptions.whileFalse(new InstantCommand(() -> elevator.elevatorStop()));
        j.oTouchpad.whileFalse(new InstantCommand(() -> elevator.elevatorStop()));

        //elevator setpoints
        // j.oUp.whileTrue(new InstantCommand(() -> elevator.elevatorL4()));
        // j.oRight.whileTrue(new InstantCommand(() -> elevator.elevatorL3()));
        // j.oLeft.whileTrue(new InstantCommand(() -> elevator.elevatorL2()));
        // j.oDown.whileTrue(new InstantCommand(() -> elevator.elevatorGround()));

<<<<<<< HEAD
<<<<<<< HEAD
        j.oOptions.onTrue(new InstantCommand(() -> elevator.manualOverride()));
        

        Mode = Elevator.elevatorManual;

        j.oUp.whileTrue(new InstantCommand(() -> elevator.L4(Elevator.elevatorManual)));
        j.oRight.whileTrue(new InstantCommand(() -> elevator.L3(Elevator.elevatorManual)));
        j.oLeft.whileTrue(new InstantCommand(() -> elevator.L2(Elevator.elevatorManual)));
        j.oDown.whileTrue(new InstantCommand(() -> elevator.elevatorGround()));
        j.dLB.whileTrue(new autoScore(drivetrain, true, elevator, coral));
        j.dRB.whileTrue(new autoScore(drivetrain, false, elevator, coral)); 

=======
=======
>>>>>>> parent of 018d848 (PoseAlignBargeAnywhere & AutoScore UPDATES)
        //if (Constants.ElevatorConstants.elevatorManual == true) {
            j.XboxUp.whileTrue(new InstantCommand(() -> elevator.elevatorL4()));
            j.XboxRight.whileTrue(new InstantCommand(() -> elevator.elevatorL3()));
            j.XboxLeft.whileTrue(new InstantCommand(() -> elevator.elevatorL2()));
            j.XboxDown.whileTrue(new InstantCommand(() -> elevator.elevatorGround()));
        //}
        // if (Constants.ElevatorConstants.elevatorManual == false) {
        //     j.XboxUp.whileTrue(new InstantCommand(() -> elevator.autoL4()));
        //     j.XboxRight.whileTrue(new InstantCommand(() -> elevator.autoL3()));
        //     j.XboxLeft.whileTrue(new InstantCommand(() -> elevator.autoL2()));
        //     j.XboxDown.whileTrue(new InstantCommand(() -> elevator.elevatorGround()));
        // }

<<<<<<< HEAD
>>>>>>> parent of 018d848 (PoseAlignBargeAnywhere & AutoScore UPDATES)
=======
>>>>>>> parent of 018d848 (PoseAlignBargeAnywhere & AutoScore UPDATES)
        j.dL3.whileTrue(new InstantCommand(() -> elevator.elevatorGround()));
        // j.dShare.whileTrue(new InstantCommand(() -> elevator.elevatorL2()));
        j.dR3.whileTrue(new InstantCommand(() -> elevator.switchElevator()));

        // j.oUp.whileFalse(new InstantCommand(() -> elevator.elevatorStop()));
        // j.oRight.whileFalse(new InstantCommand(() -> elevator.elevatorStop()));
        // j.oLeft.whileFalse(new InstantCommand(() -> elevator.elevatorStop()));
        // j.oDown.whileFalse(new InstantCommand(() -> elevator.elevatorStop())); //uncomment if we want to hold down


        //algae gripper
        //og algae gripper commands ===== y and x
        // j.oY.whileTrue(new InstantCommand( () -> algae.intake()));
        // j.oX.whileTrue(new InstantCommand( () -> algae.outtake()));
        // j.oY.whileFalse(new InstantCommand(() -> algae.stop()));
        // j.oX.whileFalse(new InstantCommand(() -> algae.stop()));

        // j.dA.whileTrue(new InstantCommand(() -> algae.outtake()));
        // j.dA.whileFalse(new InstantCommand(() -> algae.stopIntake()));

        // j.oR3.onTrue(new InstantCommand(() -> algae.gripperUp())).onFalse(new InstantCommand(() -> algae.stopVertical()));
        // j.oL3.onTrue(new InstantCommand(() -> algae.gripperDown())).onFalse(new InstantCommand(() -> algae.stopVertical()));

        Trigger oRTNew = new Trigger(() -> j.operator.getRawAxis(4) > 0.1);

        // j.oLT.whileTrue(new InstantCommand(() -> algae.verticalScore()).alongWith(new InstantCommand(() -> algae.intake()))).whileFalse(new InstantCommand(() -> algae.verticalHome()).andThen(new InstantCommand(() -> algae.stopIntake())));
        // j.oRT.whileTrue(new InstantCommand(() -> algae.verticalBarge())).onFalse(new InstantCommand(() -> algae.outtake()));
        // // oRTNew.whileTrue(new RepeatCommand(new InstantCommand(() -> algae.gripperUp(-j.operator.getRawAxis(4) * 0.2))));
        // // j.oLT.whileFalse(new InstantCommand(() -> algae.stop()));
        // j.oRT.whileFalse(new InstantCommand(() -> algae.stop()));

        // j.oL3.whileTrue(new InstantCommand(() -> algae.gripperUp()));
        // j.oR3.whileTrue(new InstantCommand(() -> algae.gripperDown()));
        // j.oL3.whileFalse(new InstantCommand(() -> algae.stopVertical()));
        // j.oR3.whileFalse(new InstantCommand(() -> algae.stopVertical()));

        j.oRT.whileTrue(new RepeatCommand(new InstantCommand(() -> algae.intake())));
        j.oLT.whileTrue(new InstantCommand(() -> algae.outtake()));
        j.oRT.whileFalse(new InstantCommand(() -> algae.stop()));
        j.oLT.whileFalse(new InstantCommand(() -> algae.stop()));

        //algae macro
        j.oY.whileTrue(new AlgaeL3(algae, elevator)).onFalse(new InstantCommand(() -> elevator.elevatorStop()));
        j.oX.whileTrue(new AlgaeL2(algae, elevator)).onFalse(new InstantCommand(() -> elevator.elevatorStop()));

        //normal coral intake
        j.oA.whileTrue(new InstantCommand(() -> coral.switchIntake())); //was: coral.switchIntake()
        j.oB.whileTrue(new InstantCommand( () -> coral.switchOuttake()));
        j.dRT.whileTrue(new InstantCommand(() -> coral.switchIntake()).unless(j.dLT));
        j.oA.whileFalse(new InstantCommand(() -> coral.stop()));
        j.oB.whileFalse(new InstantCommand(() -> coral.stop()));
        j.dRT.whileFalse(new InstantCommand(() -> coral.stop()).unless(j.oA));
        
        //floor intake
        j.dUp.whileTrue(new InstantCommand(() -> intake.up()));
        j.dDown.whileTrue(new InstantCommand(() -> intake.down()));
        j.dUp.whileFalse(new InstantCommand(() -> intake.stop()));
        j.dDown.whileFalse(new InstantCommand(() -> intake.stop()));

        //we need X and Y buttons for algae
        // j.oY.whileTrue(new InstantCommand(() -> intake.up())).onFalse(new WaitCommand(0.3).andThen(new InstantCommand(() -> intake.zeroVertical())));
        // j.oX.whileTrue(new InstantCommand(() -> intake.down()));
        // j.oY.whileFalse(new InstantCommand(() -> intake.stop()));
        // j.oX.whileFalse(new InstantCommand(() -> intake.stop()));



        //zeroing things --- driver for side intake, operator for algae gripper
        j.dPS.whileTrue(new InstantCommand(() -> intake.zeroVertical()));
<<<<<<< HEAD
        j.oPS.whileTrue(new InstantCommand(() -> intake.zeroVertical()));
=======
        // j.oPS.whileTrue(new InstantCommand(() -> intake.zeroVertical()));
        j.XboxStart.whileTrue(new InstantCommand(() -> intake.zeroVertical()));
        // if (Constants.ElevatorConstants.elevatorManual == true) {
        //     j.XboxStart.whileTrue(new InstantCommand(() -> elevator.autoScore()));
        // }
        // if (Constants.ElevatorConstants.elevatorManual == false) {
        //     j.XboxStart.whileTrue(new InstantCommand(() -> elevator.manualScore()));
        // }
<<<<<<< HEAD
>>>>>>> parent of 018d848 (PoseAlignBargeAnywhere & AutoScore UPDATES)
=======
>>>>>>> parent of 018d848 (PoseAlignBargeAnywhere & AutoScore UPDATES)
        // j.oPS.whileTrue(new InstantCommand(() -> algae.zeroVertical()));

        //operator floor intake macros
        j.oRB.whileTrue(new InstantCommand(() -> intake.lowerIntake()));
        j.oRB.whileTrue(new InstantCommand(() -> intake.intake()));
        j.oRB.whileFalse(new InstantCommand(() -> intake.intakeIdle()));
        //j.oRB.whileFalse(new InstantCommand(() -> intake.stopIntake()));
        j.oRB.whileFalse(new InstantCommand(() -> intake.raiseIntake()));

        j.oLB.whileTrue(new InstantCommand(() -> intake.scoreIntake()));
        j.oLB.whileTrue(new InstantCommand(() -> intake.outtake()));
        j.oLB.whileFalse(new InstantCommand(() -> intake.stopIntake()));
        j.oLB.whileFalse(new InstantCommand(() -> intake.raiseIntake()));

        //alignment to apriltag
        // j.dLeft.whileTrue(new DTPLeft(drivetrain)); //lb
        // j.dRight.whileTrue(new DriveToPoseBeta(drivetrain)); //old right promoted @ states
        // j.dLeft.whileTrue(new JalignLeft(drivetrain));
        // j.dRight.whileTrue(new JalignRight(drivetrain));
<<<<<<< HEAD
<<<<<<< HEAD
        j.dLB.whileTrue(new PoseAlign(drivetrain, true));
        j.dRB.whileTrue(new PoseAlign(drivetrain, false)); //new right has been demoted //its back!!
=======
        j.dLB.whileTrue(new autoScore(drivetrain, true, elevator, coral));
        j.dRB.whileTrue(new autoScore(drivetrain, false, elevator, coral)); //new right has been demoted //its back!!
>>>>>>> parent of 018d848 (PoseAlignBargeAnywhere & AutoScore UPDATES)
=======
        j.dLB.whileTrue(new autoScore(drivetrain, true, elevator, coral));
        j.dRB.whileTrue(new autoScore(drivetrain, false, elevator, coral)); //new right has been demoted //its back!!
>>>>>>> parent of 018d848 (PoseAlignBargeAnywhere & AutoScore UPDATES)

        j.dRT.and(j.dLT).whileTrue(new PoseAlignHP(drivetrain));
        // j.dB.whileTrue(new PoseAlignHP(drivetrain));
        j.dPS.and(j.dShare).whileTrue(new PoseAlignToAutoStartingPt(drivetrain));


        // j.dRight.whileTrue(new OnlyTurn2(drivetrain));
        // j.dRight.whileTrue(new TurnToAngleHolonomic(drivetrain, 60, false));
        // j.dRight.whileTrue(new TTAHolonomicAprilTag(drivetrain));
        j.oShare.whileTrue(new PoseAlignBargeAuto(drivetrain));

        j.dLeft.whileTrue(new SmartPathfinding(drivetrain));
        

        j.dShare.whileTrue(new RumbleUntimed());
        j.dX.whileTrue(new InstantCommand(() -> candle.toggleNoah()));
        // j.dOptions.whileTrue(new InstantCommand(() -> PoseAlign.printAllGoals())); //get all goalposes, used for autos

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() { 
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
