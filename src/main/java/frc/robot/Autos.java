package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.vision.DTPLeft;
import frc.robot.commands.vision.DTPLeftAuto;
import frc.robot.commands.vision.DriveToPoseBeta;
import frc.robot.commands.vision.DriveToPoseBetaAutonomous;
import frc.robot.commands.vision.JalignLeft;
import frc.robot.commands.vision.JalignRight;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class Autos {
    
    static Algae algae;
    static Coral coral;
    static Elevator elevator;
    static Intake intake;
    static Climber climber;
    static CommandSwerveDrivetrain drivetrain;

    // enum autos { 
    //     Nothing, 
    //     Calibration10, 
    //     CalibrationSquare 
    //      }


    public Autos(CommandSwerveDrivetrain drivetrain, Algae algae, Coral coral, Elevator elevator, Intake intake, Climber climber){
        this.algae = algae;
        this.coral = coral;
        this.elevator = elevator;
        this.intake = intake;
        this.climber = climber;
        this.drivetrain = drivetrain;
    }

    public void makeNamedCommands(){
        NamedCommands.registerCommand("ZeroGyro", drivetrain.runOnce(() -> drivetrain.seedFieldCentric()).withTimeout(0.05));
        NamedCommands.registerCommand("HoldIntake", new InstantCommand(() -> intake.zeroVertical()).withTimeout(0.05).andThen(new InstantCommand(() -> intake.raiseIntake())));
        NamedCommands.registerCommand("ExtendClimber", new InstantCommand(() -> climber.unclimb()).withTimeout(1).finallyDo(() -> new InstantCommand(() -> climber.stop()).withTimeout(0.02)));

        NamedCommands.registerCommand("DTPBeta", new DriveToPoseBetaAutonomous(drivetrain)); 
        NamedCommands.registerCommand("DTPLeft", new DTPLeftAuto(drivetrain));
        NamedCommands.registerCommand("JalignLeft", new JalignLeft(drivetrain));
        NamedCommands.registerCommand("JalignRight", new JalignRight(drivetrain));

        NamedCommands.registerCommand("ElevatorL4", new InstantCommand(() -> elevator.elevatorL4()));
        NamedCommands.registerCommand("ElevatorL4Delayed", new WaitCommand(2).andThen(new InstantCommand(() -> elevator.elevatorL4())));
        NamedCommands.registerCommand("ElevatorL3", new InstantCommand(() -> elevator.elevatorL3()));
        NamedCommands.registerCommand("ElevatorL2", new InstantCommand(() -> elevator.elevatorL2()));
        NamedCommands.registerCommand("ElevatorL2Algae", new InstantCommand(() -> elevator.elevatorL2Algae()));
        NamedCommands.registerCommand("ElevatorL3Algae", new InstantCommand(() -> elevator.elevatorL3Algae()));
        
        NamedCommands.registerCommand("ElevatorGround", new InstantCommand(() -> elevator.elevatorGround()));
        NamedCommands.registerCommand("ElevatorSlow", new InstantCommand(() -> elevator.autoAlgaeSlow()));

        NamedCommands.registerCommand("AlgaeIntake", new InstantCommand(() -> algae.intake()));
        NamedCommands.registerCommand("AlgaeOuttake", new InstantCommand(() -> algae.outtake()));
        NamedCommands.registerCommand("AlgaeStop", new InstantCommand(() -> algae.stop()));

        NamedCommands.registerCommand("ShootCoral", new ParallelDeadlineGroup(new WaitCommand(.15), new InstantCommand(() -> coral.switchIntakeAuto())).until(() -> coral.irOne.get() && coral.irTwo.get()).andThen(new ParallelDeadlineGroup(new WaitCommand(0.15), new InstantCommand(() -> coral.stop()))));
        NamedCommands.registerCommand("StopShooting", new InstantCommand(() -> coral.stop()));
        NamedCommands.registerCommand("IntakeHP", new InstantCommand(() -> coral.stop()).withTimeout(0.02).andThen(new InstantCommand(() -> coral.switchIntake())));
        

        new EventTrigger("ElevatorL4").whileTrue(new InstantCommand(() -> elevator.elevatorL4()));
    }

    public SendableChooser<Command> configureChooser(SendableChooser<Command> chooser){
        chooser.addOption("Left", new PathPlannerAuto("Left"));
        chooser.addOption("Right", new PathPlannerAuto("Left", true));
        chooser.addOption("Center", new PathPlannerAuto("Center"));
        chooser.addOption("C10", new PathPlannerAuto("C10"));
        chooser.addOption("Nothing", new WaitCommand(15));
        chooser.addOption("CenterMainAuto", new PathPlannerAuto("CenterMainAuto"));

        return chooser;
    }


}
