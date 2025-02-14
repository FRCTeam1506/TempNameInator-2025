package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class Constants {
    




    public class ClimberConstants{
        public static final int CLIMBER_ID = 52;
        

        public static final double CLIMB_SPEED = 0.2;
    }

    public class ElevatorConstants {
        public static final int ELEVATOR_ID = 55;
        public static final int ELEVATOR2_ID = 56;

        public static final int L1Pos = 10;
        public static final int L2Pos = 20;
        public static final int L3Pos = 30;
        public static final int L4Pos = 40;

        public static final double ELEVATOR_SPEED = 0.5;
        public static final DigitalInput LimitSwitchDIO = new DigitalInput(0);


    }

    public class OuttakeConstants {
        public static final int OUTTAKE_ID = 58;//May Change


        public static final double ScoreSpeed = 1;
        public static final double PrepSpeed = 0.2;
        public static final double RejectSpeed = -0.8;

    }


    public class AlgaeConstants {
        public static final int GripperOne_ID = 60;//May Change
        public static final int GripperTwo_ID = 61;//May Change


        public static final double gripSpeedOne = 0.3;
        public static final double dropSpeedOne = -1;


        public static final double gripSpeedTwo = 0.3;
        public static final double dropSpeedTwo = -1;

    }
}
