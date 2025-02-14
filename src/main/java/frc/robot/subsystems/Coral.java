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
import frc.robot.Constants.CoralConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;


public class Coral extends SubsystemBase {
  private TalonFX motor = new TalonFX(Constants.ElevatorConstants.ELEVATOR_ID);
  private DigitalInput irNine = new DigitalInput(CoralConstants.irInput);
  private DigitalInput irOne = new DigitalInput(CoralConstants.irOutput);


  public boolean isClicked = false;
  public boolean hasBeenClickedYet = false;
  DigitalInput input = Constants.ElevatorConstants.LimitSwitchDIO;

  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);

  

  public Coral() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motor.getConfigurator().apply(config);


  }


  public void prepCoral() {
    motor.set(CoralConstants.PrepSpeed);
  }
  
  public void intake() {
    if(!irNine.get()){
      stop();
    }
    else{
      motor.set(CoralConstants.forwardSpeed);
    }
  }

  public void reverse() {
    motor.set(CoralConstants.RejectSpeed);
  }

  public void stop() {
    motor.set(0);
    motor.stopMotor();
  }

  



  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("ir 1", irOne.get());
    SmartDashboard.putBoolean("ir 2", irNine.get());

  }
}
