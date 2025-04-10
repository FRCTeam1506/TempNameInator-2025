package frc.robot;

import java.util.Hashtable;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.wpilibj.DigitalInput;

public class Constants {

    public static double scoreSpeed = 0.8;
    public static boolean raisable;
    

    public static final Slot0Configs slot0Configs = new Slot0Configs().withKS(0.24).withKV(0.12).withKP(4.8).withKI(0).withKD(0.1);


    public class ClimberConstants{
        public static final int CLIMBER_ID = 52;

        public static final int LS_CLIMBER = 1; //limit switch
        

        public static final double CLIMB_SPEED = 0.2;
    }

    public class ElevatorConstants {
        public static final int ELEVATOR_ID = 55;
        public static final int ELEVATOR2_ID = 56;

        // public static final int L4Pos = 10;
        public static final double L2Pos = 29.5; //30 low KUweek2 //30.5 slightly high with jalign, FRCC
        public static final double L2AlgaePos = 16;
        public static final double L3Pos = 53; //53 slightly low FRCC  //53.4 too low when angled KUweek2
        public static final double L3AlgaePos = 41.25;
        public static final double L4Pos = 87; //87 prev, 87.5

        public static final double ELEVATOR_SPEED = 0.4; //0.2
        public static final double ELEVATOR_SPEED_SLOW = 0.3;

        public enum ElevatorLevel{
            Ground,
            L2,
            L3,
            L4,
        }

        public static ElevatorLevel current = ElevatorLevel.Ground;
    }

    public class CoralConstants {
        public static final int MOTOR_ID = 58;
        public static final int irInput = 3;
        public static final int irOutput = 2;
        public static final int irThree = 5;

        public static final double forwardSpeed = .5;
        public static final double PrepSpeed = 0.2;
        public static final double reverseSpeed = -0.8;
    }

    public class GroundIntakeConstants {
        public static final int VERTICAL_ID = 53;
        public static final int INTAKE_ID = 54;

        public static final int BUTTON_PORT = 4;

        public static final double groundPosition = -4.25; //-4.3
        public static final double upperPosition = 0;
        public static final double scorePosition = -.5;

        public static final double intakeSpeed = 0.8;
        public static final double outtakeSpeed = 0.6;

        public static final double holdSpeed = 0.2;
        public static final double upSpeed = 0.1; //.25
        public static final double downSpeed = 0.15; //0.3

    }


    public class AlgaeConstants {
        public static final int VERTICAL_ID = 60;//May Change
        public static final int INTAKE_ID = 61;//May Change


        public static final double intakeSpeed = 1;
        public static final double outtakeSpeed = -1;


        public static final double gripSpeedTwo = 0.3;
        public static final double dropSpeedTwo = -1;

    }

    public class AlgaeConstantsTwo {
        public static final int BottomID = 60;//May Change
        public static final int TopID = 61;//May Change


        public static final double intakeSpeed = 0.6;
        public static final double outtakeSpeed = -0.3; //-0.5

    }

    public class VisionConstants {

        public static final String LL_LEFT = "limelight-left";
        public static final String LL_CENTER = "limelight-center";

        public static final double coralStationLeftHeading = 235;
        public static final double coralStationRightHeading = 125;    
    }

    public class SwerveConstants {

        public static final double driveKP = 1.5;
        public static final double driveKI = 0;
        public static final double driveKD = 0.075;

        public static final double alignKP = 1.5; //1.5
        public static final double alignKI = 0;
        public static final double alignKD = 0.075;

        public static final double dMaxVelocity = 1;
        public static final double dMaxAccel = 2;

        public static final double tMaxVelocity = 4; //rad/s
        public static final double tMaxAccel = 1;

    }

    public class CandleConstants {
        public static final int CANDLE_ID = 62;
        public static boolean noah = false;
    }
}
