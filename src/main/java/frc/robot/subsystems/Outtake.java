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


public class Outtake extends SubsystemBase {
  private TalonFX outtakeMotor = new TalonFX(Constants.ElevatorConstants.ELEVATOR_ID);

  public boolean isClicked = false;
  public boolean hasBeenClickedYet = false;
  DigitalInput input = Constants.ElevatorConstants.LimitSwitchDIO;

  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);

  


  /** Creates a new Elevator. */
  public Outtake() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    outtakeMotor.getConfigurator().apply(config);


  }


  public void PrepCoral() {
    outtakeMotor.set(Constants.OuttakeConstants.PrepSpeed);
  }
  public void RejectCoral() {
    outtakeMotor.set(Constants.OuttakeConstants.RejectSpeed);
  }
  public void ScoreCoral() {
    outtakeMotor.set(Constants.OuttakeConstants.ScoreSpeed);
  }
  public void Stop() {
    outtakeMotor.set(0);
  }

  



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
