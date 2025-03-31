package frc.robot;

import java.util.HashMap;
import java.util.Hashtable;
import java.util.Map;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.generated.TunerConstants;
import frc.robot.util.FieldPoint;
import frc.robot.util.FieldPoly;

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
        
        
        //todo: definitely change these values for our robot.
        // X is in the normal direction of the tag, Y is parallel to the tag 
        public static final Transform2d leftBranch = new Transform2d(0.46769, -0.14993, new Rotation2d(Math.PI)); // -0.14993
        public static final Transform2d rightBranch = new Transform2d(0.46769, 0.16, new Rotation2d(Math.PI));
        public static final Transform2d reefAlgae = new Transform2d(0.5,0.0,new Rotation2d(Math.PI));


        public static double std02 = 15; //standard deviation for vision??? seems high
        public static double maxStdDeviation = .5; 
        /** cut-off tag area, don't trust past this point */
        public static final double minTagArea = 0.35; 
        public static double visionStdSlope = (maxStdDeviation-std02)/(4-.2); // from .2 to 4 // 2m to .5m // units (Deviation / Tag Area)
        public static double visionStdConstant = std02 - visionStdSlope * .2; // Units (Deviation)
    
        public static double getVisionStd(double tagArea){
            double std = visionStdSlope * tagArea + visionStdConstant;
    
            if (tagArea < minTagArea){ 
                return 9999999;
            }
            if (std < maxStdDeviation){
                return maxStdDeviation;
            }
            
            return std;
        }

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

        //on the fly path constraitns
        public static PathConstraints oTF_Constraints = new PathConstraints(5.3, 5, Math.toRadians(270), Math.toRadians(360));    
    }

    public class CandleConstants {
        public static final int CANDLE_ID = 62;
        public static boolean noah = false;
    }











    //https://github.com/TheMathWiz56/2848_Reefscape/blob/without_algae_changes/src/main/java/frc/robot/Constants.java
    // util values for the reef
    public static class reef {
        /* possible reef values l for left, r for right, L for level */
        public static enum reefSide {
            LEFT, RIGHT
        }

        public static enum reefLs {
            lL4,
            lL3,
            lL2,
            lL1,
            rL4,
            rL3,
            rL2,
            rL1,
            STOW,
            NONE
        }

        public static final Map<Integer, Integer> rMap = new HashMap<>() {
            {
                put(9, 1);
                put(10, 2);
                put(11, 3);
                put(12, 4);
                put(13, 5);
                put(14, 6);
            }
        };


        public static final Map<Integer, reefLs> lMap = new HashMap<>() {
            {
                put(1, reefLs.lL4);
                put(2, reefLs.lL3);
                put(3, reefLs.lL2);
                put(4, reefLs.lL1);
                put(5, reefLs.rL4);
                put(6, reefLs.rL3);
                put(7, reefLs.rL2);
                put(8, reefLs.rL1);
                put(22,reefLs.STOW);
                put(19, reefLs.STOW);
            }
        };

        // zones for auto driving around the reef
        public static final class driveZones {

            public static final FieldPoly reef1Zone = new FieldPoly(
                    new FieldPoint(5, 0),
                    new FieldPoint(-5, 0),
                    new FieldPoint(-5, -5),
                    new FieldPoint(5, -5));

            public static final Map<Integer, FieldPoly> fieldPolyList = new HashMap<>() {
                {
                    put(1, reef1Zone);
                }
            };
        }

        

    }

}
