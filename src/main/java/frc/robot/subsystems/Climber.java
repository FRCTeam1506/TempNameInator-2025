// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  private TalonFX climber = new TalonFX(Constants.ClimberConstants.CLIMBER_ID);

  /** Creates a new Climb. */
  public Climber() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    climber.getConfigurator().apply(config);
    climber.setPosition(0);
  }

  public void up() {
    climber.set(-ClimberConstants.CLIMB_SPEED*1.5);
  }

  public void down() {
    climber.set(ClimberConstants.CLIMB_SPEED*1.5);
  }

  public void stop(){
    climber.set(0);
    climber.stopMotor();
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
