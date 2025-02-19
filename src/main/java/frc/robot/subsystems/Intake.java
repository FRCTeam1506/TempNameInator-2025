package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GroundIntakeConstants;


public class Intake extends SubsystemBase {
  private TalonFX vertical = new TalonFX(GroundIntakeConstants.VERTICAL_ID);
  private TalonFX intake = new TalonFX(GroundIntakeConstants.INTAKE_ID);

  DigitalInput button = new DigitalInput(GroundIntakeConstants.BUTTON_PORT);

  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);

  
  public Intake() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 80;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.MotionMagic.MotionMagicCruiseVelocity = 60; // 80 rps cruise velocity
    config.MotionMagic.MotionMagicAcceleration = 160; // 160 rps/s acceleration (0.5 seconds)
    config.MotionMagic.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)

    config.Slot0 = Constants.slot0Configs;

    vertical.getConfigurator().apply(config);
    intake.getConfigurator().apply(config);

    intake.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
  }


  public void intake() {
    intake.set(GroundIntakeConstants.intakeSpeed);
  }
  public void outtake() {
    intake.set(-GroundIntakeConstants.outtakeSpeed);
  }

  public void intakeIdle(){
    intake.set(0.25);
  }

  public void up() {
    vertical.set(GroundIntakeConstants.upSpeed);
  }
  public void down() {
    vertical.set(-GroundIntakeConstants.downSpeed);
  }

  public void lowerIntake() {
    vertical.setControl(m_motmag.withPosition(GroundIntakeConstants.groundPosition));
    if(Math.abs(vertical.getTorqueCurrent().getValueAsDouble()) > 20){
      vertical.setPosition(GroundIntakeConstants.groundPosition);
      // stop();
    }
  }
  public void raiseIntake() {
    vertical.setControl(m_motmag.withPosition(0));

    if(Math.abs(vertical.getTorqueCurrent().getValueAsDouble()) > 15){
      vertical.setPosition(0);
      // stop();
    }
  }
  public void scoreIntake(){
    vertical.setControl(m_motmag.withPosition(GroundIntakeConstants.scorePosition));
  }

  public void stop() {
    vertical.set(0);
    intake.set(0);
    vertical.stopMotor();
    intake.stopMotor();
  }

  public void stopIntake() {
    intake.set(0);
    intake.stopMotor();
  }

  public void zeroVertical(){
    vertical.setPosition(0);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("torque current vertical", vertical.getTorqueCurrent().getValueAsDouble());
    SmartDashboard.putBoolean("intake button", button.get());
  }
}
