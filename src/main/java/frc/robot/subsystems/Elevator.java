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

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import java.lang.annotation.ElementType;
import java.util.Map;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;


public class Elevator extends SubsystemBase {
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
    motionMagicConfigs.MotionMagicCruiseVelocity = 160; // 80 rps cruise velocity //60 rps gets to L4 in 1.92s //100
    motionMagicConfigs.MotionMagicAcceleration = 220; // 160 rps/s acceleration (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.24; // add 0.24 V to overcome friction
    slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    // PID runs on position
    slot0Configs.kP = 2.5; //4.8
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.1;

    config.Slot0 = slot0Configs;

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


  public void elevatorStop() {
    elevator1.set(0);
    elevator2.set(0);
    elevator1.stopMotor();
    elevator2.stopMotor();
  }



  public void elevatorGround() {
    elevator1.setControl(m_motmag.withPosition(0));
    elevator2.setControl(m_motmag.withPosition(0));
  }
  public void elevatorL2() {
    elevator1.setControl(m_motmag.withPosition(ElevatorConstants.L2Pos));
    elevator2.setControl(m_motmag.withPosition(ElevatorConstants.L2Pos));
  }
  public void elevatorL3() {
    elevator1.setControl(m_motmag.withPosition(ElevatorConstants.L3Pos));
    elevator2.setControl(m_motmag.withPosition(ElevatorConstants.L3Pos));
  }
  public void elevatorL4() {
    elevator1.setControl(m_motmag.withPosition(ElevatorConstants.L4Pos));
    elevator2.setControl(m_motmag.withPosition(ElevatorConstants.L4Pos));
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



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("elevator1 position", elevator1.getRotorPosition().getValueAsDouble());
    // SmartDashboard.putNumber("elevator2 position", elevator2.getRotorPosition().getValueAsDouble());
    SmartDashboard.putBoolean("limit switch elevator", input.get()); // false when touching elevator, true when elevator is elevated
    testSwitch();

    if(Math.abs(getAvgTorqueCurrent()) > 50){
      elevatorStop();
    }
  }
}
