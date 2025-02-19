package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class Autos {
    
    static Algae algae;
    static Coral coral;
    static Elevator elevator;
    static Intake intake;
    static CommandSwerveDrivetrain drivetrain;

    // enum autos { 
    //     Nothing, 
    //     Calibration10, 
    //     CalibrationSquare 
    //      }


    public Autos(CommandSwerveDrivetrain drivetrain, Algae algae, Coral coral, Elevator elevator, Intake intake){
        this.algae = algae;
        this.coral = coral;
        this.elevator = elevator;
        this.intake = intake;
        this.drivetrain = drivetrain;
    }

    public void makeNamedCommands(){
        NamedCommands.registerCommand("ZeroGyro", drivetrain.runOnce(() -> drivetrain.seedFieldCentric()).withTimeout(0.05));
        NamedCommands.registerCommand("HoldIntake", new InstantCommand(() -> intake.zeroVertical()).withTimeout(0.05).andThen(new InstantCommand(() -> intake.raiseIntake())));

        NamedCommands.registerCommand("ElevatorL4", new InstantCommand(() -> elevator.elevatorL4()));
        NamedCommands.registerCommand("ElevatorL4Delayed", new WaitCommand(2).andThen(new InstantCommand(() -> elevator.elevatorL4())));
        NamedCommands.registerCommand("ElevatorL3", new InstantCommand(() -> elevator.elevatorL3()));
        NamedCommands.registerCommand("ElevatorL2", new InstantCommand(() -> elevator.elevatorL2()));
        NamedCommands.registerCommand("ElevatorGround", new InstantCommand(() -> elevator.elevatorGround()));

        NamedCommands.registerCommand("ShootCoral", new InstantCommand(() -> coral.justScore()));

        

        new EventTrigger("ElevatorL4").whileTrue(new InstantCommand(() -> elevator.elevatorL4()));
    }


}
