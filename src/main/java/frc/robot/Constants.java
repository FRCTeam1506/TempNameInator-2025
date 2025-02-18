package frc.robot;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.wpilibj.DigitalInput;

public class Constants {
    

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
        public static final int L2Pos = 30;
        public static final int L3Pos = 53;
        public static final int L4Pos = 87;

        public static final double ELEVATOR_SPEED = 0.2;
    }

    public class CoralConstants {
        public static final int MOTOR_ID = 58;
        public static final int irInput = 3;
        public static final int irOutput = 2;

        public static final double forwardSpeed = .5;
        public static final double PrepSpeed = 0.2;
        public static final double reverseSpeed = -0.8;
    }

    public class GroundIntakeConstants {
        public static final int VERTICAL_ID = 53;
        public static final int INTAKE_ID = 54;

        public static final int BUTTON_PORT = 4;

        public static final double groundPosition = -4.13;
        public static final double upperPosition = 0;
        public static final double scorePosition = -.5;

        public static final double intakeSpeed = 0.8;
        public static final double outtakeSpeed = 0.6;

        public static final double holdSpeed = 0.2;
        public static final double upSpeed = 0.25;
        public static final double downSpeed = 0.3;

    }


    public class AlgaeConstants {
        public static final int VERTICAL_ID = 60;//May Change
        public static final int INTAKE_ID = 61;//May Change


        public static final double intakeSpeed = 0.6;
        public static final double outtakeSpeed = -1;


        public static final double gripSpeedTwo = 0.3;
        public static final double dropSpeedTwo = -1;

    }
}
