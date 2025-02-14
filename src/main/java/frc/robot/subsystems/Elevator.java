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
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;



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
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // 80 rps cruise velocity
    motionMagicConfigs.MotionMagicAcceleration = 160; // 160 rps/s acceleration (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)
  

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
  }



  public void elevatorL1() {
    elevator1.setControl(m_motmag.withPosition(Constants.ElevatorConstants.L1Pos));
    elevator2.setControl(m_motmag.withPosition(Constants.ElevatorConstants.L1Pos));
  }
  public void elevatorL2() {
    elevator1.setControl(m_motmag.withPosition(Constants.ElevatorConstants.L2Pos));
    elevator2.setControl(m_motmag.withPosition(Constants.ElevatorConstants.L2Pos));
  }
  public void elevatorL3() {
    elevator1.setControl(m_motmag.withPosition(Constants.ElevatorConstants.L3Pos));
    elevator2.setControl(m_motmag.withPosition(Constants.ElevatorConstants.L3Pos));
  }
  public void elevatorL4() {
    elevator1.setControl(m_motmag.withPosition(20));
    elevator2.setControl(m_motmag.withPosition(20));
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



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("elevator1 position", elevator1.getRotorPosition().getValueAsDouble());
    // SmartDashboard.putNumber("elevator2 position", elevator2.getRotorPosition().getValueAsDouble());
    SmartDashboard.putBoolean("limit switch elevator", input.get()); // false when touching elevator, true when elevator is elevated
    testSwitch();
  }
}
