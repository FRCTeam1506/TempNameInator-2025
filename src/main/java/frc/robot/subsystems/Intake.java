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
import frc.robot.Constants.GroundIntakeConstants;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;



public class Intake extends SubsystemBase {
  private TalonFX vertical = new TalonFX(GroundIntakeConstants.VERTICAL_ID);
  private TalonFX intake = new TalonFX(GroundIntakeConstants.INTAKE_ID);

  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);

  
  public Intake() {
    var talonFXConfigs = new TalonFXConfiguration();


    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    vertical.getConfigurator().apply(config);
    intake.getConfigurator().apply(config);

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // 80 rps cruise velocity
    motionMagicConfigs.MotionMagicAcceleration = 160; // 160 rps/s acceleration (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)
  


  }


  public void intake() {
    intake.set(Constants.AlgaeConstants.intakeSpeed);
  }
  public void outtake() {
    intake.set(Constants.AlgaeConstants.outtakeSpeed);
  }

  public void up() {
    vertical.set(0.2);
  }
  public void down() {
    vertical.set(-0.2);
  }


  public void lowerIntake() {
    intake.setControl(m_motmag.withPosition(0.5));
  }
  public void raiseIntake() {
    intake.setControl(m_motmag.withPosition(0));
  }

  public void stop() {
    vertical.set(0);
    intake.set(0);
    vertical.stopMotor();
    intake.stopMotor();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
