package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Vision;


public class systemTest extends SubsystemBase {

    public final Climber climber = new Climber();
    public final Elevator elevator = new Elevator();
    public final Algae algae = new Algae();
    public final Coral coral = new Coral();
    public final Intake intake = new Intake();
    public final Vision vision = new Vision();
  
  
  public systemTest() {

  }



  public void runTest(int running) {
    switch (running) {
        case 10:
            coral.justScore();
            new WaitCommand(1);
            coral.stop();
            new WaitCommand(0.1);
            running = 20;
        case 20:
            coral.reverse();
            new WaitCommand(1);
            coral.stop();
            new WaitCommand(0.1);
            running = 30;
        case 30:
            algae.gripperDown();
            new WaitCommand(0.1);
            algae.stop();
            running = 40;
        case 40: 
            algae.intake();
            new WaitCommand(0.75);
            algae.stop();
            new WaitCommand(0.1);
            running = 50; 
        case 50:
            algae.outtake();
            new WaitCommand(0.75);
            algae.stop();
            new WaitCommand(0.1);
            running = 60;
        case 60:
            algae.gripperUp();
            new WaitCommand(0.2);
            algae.stop();
            new WaitCommand(0.1);
            running = 70;
        case 70:
            intake.lowerIntake();
            new WaitCommand(0.3);
            intake.stop();
            new WaitCommand(0.1);
            running = 80;
        case 80:
            intake.intake();
            new WaitCommand(1);
            intake.stop();
            new WaitCommand(0.1);
            running = 90;
        case 90:
            intake.outtake();
            new WaitCommand(1);
            intake.stop();
            new WaitCommand(0.1);
            running = 100;
        case 100:
            intake.raiseIntake();
            new WaitCommand(0.3);
            intake.stop();
            new WaitCommand(0.1);
            running = 110;
        case 110:
            elevator.raiseToPower(0.2);
            new WaitCommand(0.3);
            elevator.elevatorStop();
            running = 120;
        case 120:
            elevator.lowerToPower(-0.2);
            new WaitCommand(0.2);
            elevator.elevatorStop();
            running = 0;
    }
  }

  @Override
  public void periodic() {

  }
}
