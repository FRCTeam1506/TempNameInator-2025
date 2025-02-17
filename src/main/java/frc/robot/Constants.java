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
        public static final double RejectSpeed = -0.8;
    }

    public class GroundIntakeConstants {
        public static final int VERTICAL_ID = 53;
        public static final int INTAKE_ID = 54;

        public static final double lowerPosition = 0;
        public static final double upperPosition = 0;
    }


    public class AlgaeConstants {
        public static final int VERTICAL_ID = 60;//May Change
        public static final int INTAKE_ID = 61;//May Change


        public static final double intakeSpeed = 0.3;
        public static final double outtakeSpeed = -1;


        public static final double gripSpeedTwo = 0.3;
        public static final double dropSpeedTwo = -1;

    }
}
