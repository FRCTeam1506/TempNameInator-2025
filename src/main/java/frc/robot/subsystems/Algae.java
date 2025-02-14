// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;



public class Algae extends SubsystemBase {
  private TalonFX algaeGripperOne = new TalonFX(Constants.AlgaeConstants.GripperOne_ID);//CHECK IF ID IS CORRECT
  private TalonFX algaeGripperTwo = new TalonFX(Constants.AlgaeConstants.GripperTwo_ID); //CHECK IF ID IS CORRECT


  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);

  


  /** Creates a new Elevator. */
  public Algae() {
    var talonFXConfigs = new TalonFXConfiguration();


    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    algaeGripperOne.getConfigurator().apply(config);
  

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // 80 rps cruise velocity
    motionMagicConfigs.MotionMagicAcceleration = 160; // 160 rps/s acceleration (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)
  


  }


  public void GripAlgae() {
    algaeGripperOne.set(Constants.AlgaeConstants.gripSpeedOne);
  }
  public void DropAlgae() {
    algaeGripperOne.set(Constants.AlgaeConstants.dropSpeedOne);
  }
  public void StopAlgae() {
    algaeGripperOne.set(0);
  }

  public void gripperLower() {
    //algaeGripperTwo.setControl(m_motmag.withPosition(0.5));
    algaeGripperTwo.set(0.2);
  }
  public void gripperRaise() {
    //algaeGripperTwo.setControl(m_motmag.withPosition(0.5));
    algaeGripperTwo.set(-0.2);
  }


  public void gripperLowerToPos() {
    algaeGripperTwo.setControl(m_motmag.withPosition(0.5));
  }
  public void gripperRaiseToPos() {
    algaeGripperTwo.setControl(m_motmag.withPosition(0));
  }

  



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
