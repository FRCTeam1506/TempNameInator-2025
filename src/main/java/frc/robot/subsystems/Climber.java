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
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  private TalonFX motor = new TalonFX(Constants.ClimberConstants.CLIMBER_ID);
  DigitalInput ls = new DigitalInput(ClimberConstants.LS_CLIMBER);

  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);

  /** Creates a new Climb. */
  public Climber() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 120; //80

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.MotionMagic.MotionMagicCruiseVelocity = 60; // 80 rps cruise velocity
    config.MotionMagic.MotionMagicAcceleration = 160; // 160 rps/s acceleration (0.5 seconds)
    config.MotionMagic.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)

    config.Slot0 = Constants.slot0Configs;

    motor.getConfigurator().apply(config);
    motor.setPosition(0);
  }

  public void unclimb() {
    motor.set(-ClimberConstants.CLIMB_SPEED*1.5);
  }

  public void climb() {
    if(ls.get()){
      motor.set(ClimberConstants.CLIMB_SPEED); //used to be *1.5 but not anymore
    }
  }

  public void turboClimb() {
    if(ls.get()){
      motor.set(ClimberConstants.CLIMB_SPEED * 2.66);
    }
  }


  public void stop(){
    motor.set(0);
    motor.stopMotor();
  }

  public void switchClicked(){
    motor.setPosition(0);
    motor.setControl(m_motmag.withPosition(2));
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Climber ls", ls.get());

    if(!ls.get()){
      stop();
      // switchClicked();
    }
  }
}
