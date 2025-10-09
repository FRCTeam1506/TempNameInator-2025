// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants.ElevatorLevel;


public class Elevator extends SubsystemBase {
  public static boolean elevatorManual = false;

  public static int autoScorePos;

  private TalonFX elevator1 = new TalonFX(Constants.ElevatorConstants.ELEVATOR_ID);
  private TalonFX elevator2 = new TalonFX(Constants.ElevatorConstants.ELEVATOR2_ID);
  public boolean isClicked = false;
  DigitalInput input = new DigitalInput(0);

  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);

  


  /** Creates a new Elevator. */
  public Elevator() {
    var talonFXConfigs = new TalonFXConfiguration();

    // talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    elevator1.getConfigurator().apply(config);
    elevator2.getConfigurator().apply(config);

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 220; // 80 rps cruise velocity //60 rps gets to L4 in 1.92s //100 //160 //220 before 3/20 bc elevator maltensioned //220 FRCC
    motionMagicConfigs.MotionMagicAcceleration = 260; // 160 rps/s acceleration (0.5 seconds) //220
    motionMagicConfigs.MotionMagicJerk = 3200; // 1600 rps/s^2 jerk (0.1 seconds)

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.24; // add 0.24 V to overcome friction
    slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    // PID runs on position
    slot0Configs.kP = 2.5; //4.8
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.1;

    config.Slot0 = slot0Configs;

    //TODO: ADD PRO LICENSE TO MOTORS
    m_motmag.EnableFOC = true; //will the elevator go faster?

    elevator1.getConfigurator().apply(motionMagicConfigs);
    elevator2.getConfigurator().apply(motionMagicConfigs);
    elevator1.getConfigurator().apply(slot0Configs);
    elevator2.getConfigurator().apply(slot0Configs); 
  }


  public void elevatorUp() {
    //Put In Position Limit
    elevator1.set(Constants.ElevatorConstants.ELEVATOR_SPEED);
    elevator2.set(Constants.ElevatorConstants.ELEVATOR_SPEED);
    
  }

  public void elevatorDown() {
    if (!isClicked) {
      elevator1.set(-Constants.ElevatorConstants.ELEVATOR_SPEED);
      elevator2.set(-Constants.ElevatorConstants.ELEVATOR_SPEED);
    }
  }

  public void autoAlgaeSlow() {
    elevator1.set(Constants.ElevatorConstants.ELEVATOR_SPEED_SLOW);
    elevator2.set(Constants.ElevatorConstants.ELEVATOR_SPEED_SLOW);
  }


  public void elevatorStop() {
    elevator1.set(0);
    elevator2.set(0);
    elevator1.stopMotor();
    elevator2.stopMotor();
  }



  public void elevatorGround() {
    elevator1.setControl(m_motmag.withPosition(0));
    elevator2.setControl(m_motmag.withPosition(0));

    Constants.scoreSpeed = 0.65;
    ElevatorConstants.current = ElevatorConstants.ElevatorLevel.Ground;
  }

  public void elevatorL2() {
    elevator1.setControl(m_motmag.withPosition(ElevatorConstants.L2Pos));
    elevator2.setControl(m_motmag.withPosition(ElevatorConstants.L2Pos));

    Constants.scoreSpeed = 0.5; //TUNE THIS
    ElevatorConstants.current = ElevatorConstants.ElevatorLevel.L2;
  }
  public void elevatorL2Algae() {
    elevator1.setControl(m_motmag.withPosition(ElevatorConstants.L2AlgaePos));
    elevator2.setControl(m_motmag.withPosition(ElevatorConstants.L2AlgaePos));

    Constants.scoreSpeed = 0.5; //TUNE THIS
    ElevatorConstants.current = ElevatorConstants.ElevatorLevel.L2;
  }

  public void elevatorL3Algae() {
    elevator1.setControl(m_motmag.withPosition(ElevatorConstants.L3AlgaePos));
    elevator2.setControl(m_motmag.withPosition(ElevatorConstants.L3AlgaePos));

    Constants.scoreSpeed = 0.5; //TUNE THIS
    ElevatorConstants.current = ElevatorConstants.ElevatorLevel.L2;
  }

  public void elevatorL3() {
    elevator1.setControl(m_motmag.withPosition(ElevatorConstants.L3Pos));
    elevator2.setControl(m_motmag.withPosition(ElevatorConstants.L3Pos));

    Constants.scoreSpeed = 0.5; //TUNE THIS
    ElevatorConstants.current = ElevatorConstants.ElevatorLevel.L3;
  }

  public void elevatorL4() {
    elevator1.setControl(m_motmag.withPosition(ElevatorConstants.L4Pos));
    elevator2.setControl(m_motmag.withPosition(ElevatorConstants.L4Pos));

    Constants.scoreSpeed = 0.65; //0.65 og speed
    ElevatorConstants.current = ElevatorConstants.ElevatorLevel.L4;
  }


  public void switchElevator(){

    if(ElevatorConstants.current == ElevatorLevel.Ground){
      elevatorL2();
    }
    else if(ElevatorConstants.current == ElevatorLevel.L2){
      elevatorL3();
    }
    else if(ElevatorConstants.current == ElevatorLevel.L3){
      elevatorL4();
    }
  }

  public void manualScore() {
    elevatorManual = true;
  }
  public void autoScore() {
    elevatorManual = false;
  }

  public void startNotPushed() {
    Constants.ElevatorConstants.startPressed = false;
  }

  public void manualOverride() {
    if (elevatorManual == true) {
      elevatorManual = false;
    } else if (elevatorManual == false) {
      elevatorManual = true;
    }
  }

  public void L4(Boolean auto) {
    if (auto == false) {
      elevator1.setControl(m_motmag.withPosition(ElevatorConstants.L4Pos));
      elevator2.setControl(m_motmag.withPosition(ElevatorConstants.L4Pos));
    } else if (auto == true) {
      autoScorePos = 4;
    }
  }
  public void L3(Boolean auto) {
    if (auto == false) {
      elevator1.setControl(m_motmag.withPosition(ElevatorConstants.L3Pos));
      elevator2.setControl(m_motmag.withPosition(ElevatorConstants.L3Pos));
    } else if (auto == true) {
      autoScorePos = 3;
    }
  }
  public void L2(Boolean auto) {
    if (auto == false) {
      elevator1.setControl(m_motmag.withPosition(ElevatorConstants.L2Pos));
      elevator2.setControl(m_motmag.withPosition(ElevatorConstants.L2Pos));
    } else if (auto == true) {
      autoScorePos = 2;
    }
  }





  public void raiseToPower(double power) {
    elevator1.set(power);
    elevator2.set(power);
  }
  public void lowerToPower(double power) {
    if (!input.get()) {
      elevator1.set(0);
      elevator2.set(0);
    } else {
      elevator1.set(power);
      elevator2.set(power);
    }
  }



  public void testSwitch(){

    if(!input.get()){
      elevator1.setPosition(0);
      elevator2.setPosition(0);
      isClicked = true;
    }
    else{
      isClicked = false;
    }
  }

  public double getAvgTorqueCurrent(){
    return (elevator1.getTorqueCurrent().getValueAsDouble() + elevator2.getTorqueCurrent().getValueAsDouble())/2;
  }

  public double getPosition(){
    return (elevator1.getPosition().getValueAsDouble() + elevator2.getPosition().getValueAsDouble()) / 2;
  }

  public static void autoL4() {
    autoScorePos = 4;
  }
  public static void autoL3() {
    autoScorePos = 3;
  }
  public static void autoL2() {
    autoScorePos = 2;
  }



  @Override
  public void periodic() {

    //System.out.println(elevatorManual);
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("elevator1 position", elevator1.getRotorPosition().getValueAsDouble());
    // SmartDashboard.putNumber("elevator2 position", elevator2.getRotorPosition().getValueAsDouble());
    SmartDashboard.putBoolean("limit switch elevator", input.get()); // false when touching elevator, true when elevator is elevated
    testSwitch();

    if(Math.abs(getAvgTorqueCurrent()) > 100){
      elevatorStop();
      System.out.println("Elevator Torque Too High!!! Stopped.");
    }
  }
}
