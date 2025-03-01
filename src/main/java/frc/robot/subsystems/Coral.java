// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CoralConstants;


public class Coral extends SubsystemBase {
  private TalonFX motor = new TalonFX(Constants.CoralConstants.MOTOR_ID);
  public static DigitalInput irTwo = new DigitalInput(CoralConstants.irInput);
  public static DigitalInput irOne = new DigitalInput(CoralConstants.irOutput);

  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);

  boolean starting;

  enum Position{
    INTAKE,
    OUTTAKE,
    MOVE_CORAL_DOWN,
    INTAKE_AUTO,
    STOP
  }

  private Position currentPosition;
  

  public Coral() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    var motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 60; // 80 rps cruise velocity
    motionMagicConfigs.MotionMagicAcceleration = 160; // 160 rps/s acceleration (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)
    config.MotionMagic = motionMagicConfigs;

    config.Slot0 = Constants.slot0Configs;

    motor.getConfigurator().apply(config);
 
    currentPosition = Position.STOP;
  }


  public void prepCoral() {
    motor.set(CoralConstants.PrepSpeed);
  }
  
  //if ir1 is true -- no coral
  //if ir1 is false -- coral
  public void intake(boolean starting){

    if(starting == irOne.get()){
      motor.set(CoralConstants.forwardSpeed);
    }
    else if(starting == irTwo.get()){
      motor.set(CoralConstants.forwardSpeed/3);
    }
    else{
      motor.setPosition(0);
      currentPosition = Position.MOVE_CORAL_DOWN;
      // currentPosition = Position.STOP; //we hav to uncomment above line when there is only one ir
    }
  }

  public void justScore(){
    motor.set(CoralConstants.forwardSpeed);
  }

  public void moveCoralDown(){
    motor.setControl(m_motmag.withPosition(2)); //6 //7.6 before adding 2nd ir
  }

  public void reverse() {
    motor.set(CoralConstants.reverseSpeed);
  }

  public void stop() {
    motor.set(0);
    motor.stopMotor();
    currentPosition = Position.STOP;
  }

  public void switchIntake(){
    starting = irOne.get();
    currentPosition = Position.INTAKE;
  }

  public void switchOuttake(){
    currentPosition = Position.OUTTAKE;
  }

  public void switchIntakeAuto(){
    currentPosition = Position.INTAKE_AUTO;
  }

  



  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("ir 1", irTwo.get()); //lower in mechanism -- irTwo
    SmartDashboard.putBoolean("ir 2", irOne.get()); //higher in mechanism -- irOne

    // System.out.println("1: " + irOne.get() + ", 2: " + irTwo.get());

    //switch case?
    if(currentPosition == Position.INTAKE){
      intake(starting);
    }
    else if(currentPosition == Position.OUTTAKE){
      reverse();
    }
    else if(currentPosition == Position.MOVE_CORAL_DOWN){
      moveCoralDown();
    }
    else if(currentPosition == Position.INTAKE_AUTO){
      justScore();
    }
    else{
      stop();
    }

  }
}
