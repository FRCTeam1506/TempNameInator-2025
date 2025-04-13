// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;



public class Algae extends SubsystemBase {
  private TalonFX top = new TalonFX(Constants.AlgaeConstantsTwo.TopID);
  private TalonFX bottom = new TalonFX(Constants.AlgaeConstantsTwo.BottomID);


  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);
  final DutyCycleOut m_dutyCycle = new DutyCycleOut(0.0);


  /** Creates a new Elevator. */
  public Algae() {
    var talonFXConfigs = new TalonFXConfiguration();


    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 110;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotionMagic.MotionMagicCruiseVelocity = 120; // 80 rps cruise velocity
    config.MotionMagic.MotionMagicAcceleration = 160; // 160 rps/s acceleration (0.5 seconds)
    config.MotionMagic.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)
    config.Slot0 = Constants.slot0Configs;

    m_motmag.EnableFOC = true;
  
    top.getConfigurator().apply(config);
    bottom.getConfigurator().apply(config);
  }




  public void intake() {

    if(top.getTorqueCurrent().getValueAsDouble() < 50){
      top.set(Constants.AlgaeConstantsTwo.intakeSpeed);
      bottom.set(-Constants.AlgaeConstantsTwo.intakeSpeed);
    }
    else{
      top.set(Constants.AlgaeConstantsTwo.intakeSpeed / 3);
      bottom.set(-Constants.AlgaeConstantsTwo.intakeSpeed /3);
    }
  }
  public void outtake() {
    top.set(Constants.AlgaeConstantsTwo.outtakeSpeed);
    bottom.set(-Constants.AlgaeConstantsTwo.outtakeSpeed);
  }
  public void outtakeAuto(){

    if(DriverStation.getAlliance().get().equals(Alliance.Red)){
    top.set(-0.45);
    bottom.set(0.45);
    }
    else{ //more powerful on blue alliance because it is angled -- not completely flat
      top.set(-0.85);
      bottom.set(0.85);
    }
  }
  public void stop() {
    top.set(0);
    bottom.set(0);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
