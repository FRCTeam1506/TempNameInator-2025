// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.LimelightHelpers;
// import frc.robot.Constants.CandleConstants;

// import com.ctre.phoenix.led.Animation;
// import com.ctre.phoenix.led.CANdle;
// import com.ctre.phoenix.led.CANdleConfiguration;
// import com.ctre.phoenix.led.ColorFlowAnimation;
// import com.ctre.phoenix.led.RainbowAnimation;
// import com.ctre.phoenix.led.SingleFadeAnimation;
// import com.ctre.phoenix.led.CANdle.LEDStripType;
// import com.ctre.phoenix.led.StrobeAnimation;


// public class Candle extends SubsystemBase {
//   /** Creates a new Candle. */
//   CANdle candle = new CANdle(CandleConstants.CANDLE_ID);

//   int[] orange = {0, 0, 255};
//   int[] green = {0, 191, 0};
//   int[] red = {191, 0, 0};


//   public Candle() {
//     CANdleConfiguration config = new CANdleConfiguration();
//     config.stripType = LEDStripType.RGB; // set the strip type to RGB
//     config.brightnessScalar = 1; // dim the LEDs to half brightness
//     candle.configAllSettings(config);
//   }

//   public void orange(){
//     //is actually blue
//     stopGSA();
//     candle.setLEDs(orange[0], orange[1], orange[2]);
//     // CandleConstants.note = true;
    
//   }

//   public void strobeOrange(){
//     stopGSA();
//     StrobeAnimation strobe = new StrobeAnimation(orange[0], orange[1], orange[2]);
//     candle.animate(strobe);
//   }

//   public void white(){
//     stopGSA();
//     candle.setLEDs(255, 255, 255);
//   }

//   public void green(){
//     stopGSA();
//     candle.setLEDs(green[0], green[1], green[2]);
//   }

//   public void strobeGreen(){
//     stopGSA();
//     StrobeAnimation strobe = new StrobeAnimation(green[0], green[1], green[2]);
//     candle.animate(strobe);
//   }

//   public void red(){
//     stopGSA();
//     candle.setLEDs(red[0], red[1], red[2]);
//   }

//   public void strobeRed(){
//     stopGSA();
//     // StrobeAnimation strobe = new StrobeAnimation(red[0], red[1], red[2]);
//     // strobe.setSpeed(1);
//     ColorFlowAnimation flow = new ColorFlowAnimation(red[0], red[1], red[2]);
//     flow.setSpeed(5);
//     candle.animate(flow);
//   }

//   public void greenBlinkAnimation(){
//     candle.animate(new SingleFadeAnimation(1, 50, 10));
//   }

//   public void gsa(){
//     RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, 64);
//     candle.animate(rainbowAnim);
//   }

//   public void stopGSA(){
//     candle.animate(null);
//   }


//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     if(Constants.CandleSubsystem.noah){
//       gsa();
//     }
//     else if(!Constants.IntakeSubsystem.irNine.get()){
//         if(Vision.shooterID == 4 || Vision.shooterID == 7){
//           if(Math.abs(Vision.x) < 3.5){
//             greenBlinkAnimation();
//           }
//           else{
//             green();
//           }
//         }else{
//           orange();//blue
//         }
//     }
//     else if(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight").tagCount > 0){
//       white();
//     }
//     else{
//       red();
//     }
//   }

// }