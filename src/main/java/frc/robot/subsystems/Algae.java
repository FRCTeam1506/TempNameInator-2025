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

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;



public class Algae extends SubsystemBase {
  private TalonFX vertical = new TalonFX(Constants.AlgaeConstants.VERTICAL_ID);
  private TalonFX intake = new TalonFX(Constants.AlgaeConstants.INTAKE_ID);


  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);
  final DutyCycleOut m_dutyCycle = new DutyCycleOut(0.0);


  /** Creates a new Elevator. */
  public Algae() {
    var talonFXConfigs = new TalonFXConfiguration();


    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 110;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotionMagic.MotionMagicCruiseVelocity = 80; // 80 rps cruise velocity
    config.MotionMagic.MotionMagicAcceleration = 160; // 160 rps/s acceleration (0.5 seconds)
    config.MotionMagic.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)
    config.Slot0 = Constants.slot0Configs;

    m_motmag.EnableFOC = true;
  
    vertical.getConfigurator().apply(config);
    intake.getConfigurator().apply(config);


  }


  public void intake() {
    intake.set(Constants.AlgaeConstants.intakeSpeed);
  }
  public void outtake() {
    intake.set(Constants.AlgaeConstants.outtakeSpeed);
  }

  public void gripperUp() {
    //algaeGripperTwo.setControl(m_motmag.withPosition(0.5));
    vertical.set(0.2);
  }

  public void gripperUp(double speed){
    vertical.set(0.8 * speed);
  }

  public void gripperDown() {
    //algaeGripperTwo.setControl(m_motmag.withPosition(0.5));
    vertical.set(-0.2);
  }


  public void verticalScore() {
    vertical.setControl(m_motmag.withPosition(-2.25)); //-2.25
  }
  public void verticalBarge(){
    vertical.setControl(m_motmag.withPosition(-0.43));
  }
  public void verticalHome() {
    vertical.setControl(m_motmag.withPosition(0));
  }

  public void stop() {
    vertical.set(0);
    intake.set(0);
    vertical.stopMotor();
    intake.stopMotor();
  }

  public void stopVertical(){
    vertical.set(0);
    vertical.stopMotor();
  }

  public void dutyCycleTest(){

    //https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/api-usage/actuator-limits.html

    boolean forwardLimit = Math.abs(vertical.getTorqueCurrent().getValueAsDouble()) > 50;

    vertical.setControl(m_dutyCycle.withOutput(0.5)
      .withLimitForwardMotion(forwardLimit));
      // .withLimitReverseMotion(m_reverseLimit.get()));
  }

  public void stopIntake(){
    intake.set(0);
    intake.stopMotor();
  }

  public void zeroVertical(){
    vertical.setPosition(0);
    System.out.println("Algae Motor Zeroed!");
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // if(Math.abs(vertical.getPosition().getValueAsDouble()) < 0.5 && vertical.getTorqueCurrent().getValueAsDouble() > 20){
    //   vertical.setPosition(0);
    // }
    if(vertical.getTorqueCurrent().getValueAsDouble() > 30 && vertical.getPosition().getValueAsDouble() > 0.5){
      vertical.setPosition(-2.25);
    }
  }
}
