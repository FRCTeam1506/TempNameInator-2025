package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.j;
import frc.robot.Constants.CandleConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.controllers.Rumble1;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix.led.StrobeAnimation;


public class Candle extends SubsystemBase {
  /** Creates a new Candle. */
  CANdle candle = new CANdle(CandleConstants.CANDLE_ID);

  int[] orange = {0, 0, 255};
  int[] green = {0, 191, 0};
  int[] red = {191, 0, 0};

  public static CANrange range = new CANrange(45);
  CommandSwerveDrivetrain drivetrain;

  public Candle(CommandSwerveDrivetrain drivetrain) {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.BRG; // set the strip type to RGB
    config.brightnessScalar = 1; // dim the LEDs to half brightness
    candle.configAllSettings(config);

    CANrangeConfiguration rangeConfig = new CANrangeConfiguration();
    // rangeConfig.ProximityParams = new ProximityParamsConfigs().withProximityHysteresis(0.4).withProximityThreshold(0.1);
    range.getConfigurator().apply(rangeConfig);

    this.drivetrain = drivetrain;
  }

  public void orange(){
    //is actually blue
    stopGSA();
    candle.setLEDs(orange[0], orange[1], orange[2]);
    // CandleConstants.note = true;
    
  }

  public void strobeOrange(){
    stopGSA();
    StrobeAnimation strobe = new StrobeAnimation(orange[0], orange[1], orange[2]);
    candle.animate(strobe);
  }

  public void white(){
    stopGSA();
    candle.setLEDs(255, 255, 255);
  }

  public void strobeWhite(){
    stopGSA();
    StrobeAnimation strobe = new StrobeAnimation(255, 255, 255);
    candle.animate(strobe);
  }

  public void fire(){
    stopGSA();
    FireAnimation fire = new FireAnimation(1, 0.5, 64, 0.7, 0.2);
    candle.animate(fire);
  }


  public void hotPink(){
    stopGSA();
    candle.setLEDs(255, 105, 180);
  }

  public void green(){
    stopGSA();
    candle.setLEDs(green[0], green[1], green[2]);
  }

  public void strobeGreen(){
    stopGSA();
    StrobeAnimation strobe = new StrobeAnimation(green[0], green[1], green[2]);
    candle.animate(strobe);
  }

  public void red(){
    stopGSA();
    candle.setLEDs(red[0], red[1], red[2]);
  }

  public void strobeRed(){
    stopGSA();
    // StrobeAnimation strobe = new StrobeAnimation(red[0], red[1], red[2]);
    // strobe.setSpeed(1);
    ColorFlowAnimation flow = new ColorFlowAnimation(red[0], red[1], red[2]);
    flow.setSpeed(5);
    candle.animate(flow);
  }

  public void greenBlinkAnimation(){
    candle.animate(new SingleFadeAnimation(1, 50, 10));
  }

  public void gsa(){
    RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, 64);
    candle.animate(rainbowAnim);
  }

  public void stopGSA(){
    candle.animate(null);
  }

  public void toggleNoah(){
    Constants.CandleConstants.noah = !Constants.CandleConstants.noah;
  }




  @Override
  public void periodic() {

    // This method will be called once per scheduler run

    // System.out.println(range.getDistance().getValueAsDouble());
    SmartDashboard.putNumber("Canrange value: ", range.getDistance().getValueAsDouble());

    if(Constants.CandleConstants.noah || DriverStation.isAutonomousEnabled()){
      gsa();
    }
    else if(!Coral.irTwo.get()){
      // if(LimelightHelpers.getTV(VisionConstants.LL_CENTER) || LimelightHelpers.getTV(VisionConstants.LL_LEFT)){
      //   hotPink();
      // }
      // else{
      //   green();
      // }

      if(range.getDistance().getValueAsDouble() > 0.31 && range.getDistance().getValueAsDouble() < .5 && (j.dRB.getAsBoolean() || j.dLB.getAsBoolean()) && (Math.abs(drivetrain.getState().Speeds.vxMetersPerSecond) + Math.abs(drivetrain.getState().Speeds.vyMetersPerSecond) < 0.25)){
        // fire();
        // gsa();
        // j.driverRumble.setRumble(RumbleType.kBothRumble, 1);
        // j.operatorRumble.setRumble(RumbleType.kBothRumble, 0.5);
        CommandScheduler.getInstance().schedule(new Rumble1());
      }
      else{
        green();
        j.driverRumble.setRumble(RumbleType.kBothRumble, 0);
        j.operator.setRumble(RumbleType.kBothRumble, 0);

      }



      // green();
    }
    else if(!Coral.irOne.get()){
        greenBlinkAnimation();
    }
    // else if(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").tagCount > 0){
    //   white();
    // }
    else{
      red();
    }




    if(DriverStation.getMatchTime() < 21 && DriverStation.getMatchTime() > 19){
      j.driverRumble.setRumble(RumbleType.kBothRumble, 0.5);
      j.operator.setRumble(RumbleType.kBothRumble, 0.5);
    }
    else{
      j.driverRumble.setRumble(RumbleType.kBothRumble, 0);
      j.operator.setRumble(RumbleType.kBothRumble, 0);
    }

  }

}