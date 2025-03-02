package frc.robot.commands.vision;

import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class align3dproper extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric m_alignRequest;

    private final ProfiledPIDController thetaController;
    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;


    ShuffleboardTab vision = Shuffleboard.getTab("vision");

    //From tunerconsts and robtocontainer.java
    private static final double MAX_AIM_VELOCITY = 1.5; //0.5*Math.PI; // radd/s
    private static final double MAX_AIM_ACCELERATION =  1.5;//Math.PI / 2; // rad/s^2
    private static final double MAX_RANGE_VELOCITY = 0.1; // m/s  //2.0
    private static final double MAX_RANGE_ACCELERATION = 1.6; // m/2^s //1.5
  
    // Todo - Tune later
    private static final double AIM_P = 5; //Proprotinal  //5
    private static final double AIM_I = 0.015; //Gradual corretction  //0.01
    private static final double AIM_D = 0.15;//0.05; //Smooth oscilattions  //0.2


    private static final double DIS_P = 8; //Proprotinal  //5
    private static final double DIS_I = 0; //Gradual corretction  //0.01
    private static final double DIS_D = 0.1;//0.05; //Smooth oscilattions  //0.2

    private static final double X_P = 1.25; //Proprotinal  //5
    private static final double X_I = 0; //Gradual corretction  //0.01
    private static final double X_D = .30;//0.05; //Smooth oscilattions  //0.2


    private static final double HOLD_DISTANCE = 1; // y hold distance
    private final SwerveRequest.ApplyRobotSpeeds request;


    public align3dproper(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        request = new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0, 0, 0));
        this.m_alignRequest = new SwerveRequest.RobotCentric().withDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1).withRotationalDeadband(0.1);
        
        Constraints cont = new Constraints(MAX_RANGE_VELOCITY, MAX_RANGE_ACCELERATION);
        thetaController = new ProfiledPIDController(AIM_P, AIM_I, AIM_D, new TrapezoidProfile.Constraints(MAX_AIM_VELOCITY, MAX_AIM_ACCELERATION));
        xController = new ProfiledPIDController(X_P, X_I, X_D, cont); //new TrapezoidProfile.Constraints(MAX_RANGE_VELOCITY, MAX_RANGE_ACCELERATION)
        yController = new ProfiledPIDController(DIS_P, DIS_I, DIS_D, cont); //cont

        thetaController.enableContinuousInput(-Math.PI, Math.PI); //Wrpa from -pi to ip    
    }

    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(VisionConstants.LL_CENTER, 0);
        
        // aimController.reset(LimelightHelpers.getTX(Constants.LIMELIGHT_NAME));
        // rangeController.reset(LimelightHelpers.getDistance(this.pipelineID,Constants.REEF_APRILTAG_HEIGHT)); //Init dist

        boolean tagExist = LimelightHelpers.getTV(VisionConstants.LL_CENTER);
        if(tagExist) {
        double theta = LimelightHelpers.getTX(VisionConstants.LL_CENTER);        
        
        double x = LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_CENTER)[0];
        double y = LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_CENTER)[2];

        // xController.reset(x);
        // yController.reset(y);

        // xController.reset(0);
        // yController.reset(0);

        }
        // else{
        //     xController.reset(0);
        //     yController.reset(0);
    
        // }
        
        
        thetaController.setGoal(0); // tx=0 is centered
        xController.setGoal(0);
        yController.setGoal(HOLD_DISTANCE);
    }

    @Override
    public void execute() {
        boolean tagExist = LimelightHelpers.getTV(VisionConstants.LL_CENTER);
        if(tagExist) {
        double theta = LimelightHelpers.getTX(VisionConstants.LL_CENTER);        
        
        double x = LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_CENTER)[0];
        double y = LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_CENTER)[2];

        // double currentDistance = Constants.pythagoreanTheorem(llvalue_x, llvalue_y);

        double rotationOutput = thetaController.calculate(Math.toRadians(theta));
        double xOutput = xController.calculate(x); //Was just (x).
        double yOutput = yController. calculate(y);
        //SmartDashboard.putNumberArray("x and y output", new double[]{xOutput, yOutput});
        System.out.println("X Pos" + xOutput + "Y Pos" + yOutput);


        // Translation2d translation = new Translation2d(xOutput, yOutput);

        drivetrain.setControl(m_alignRequest
            .withVelocityX(yOutput)//forwards and backwards? YES
            .withVelocityY(-xOutput) //-xOutput
            .withRotationalRate(rotationOutput));
        }

        //was setting velocity directly to limelight offsets and it WORKED
        // drivetrain.setControl(m_alignRequest
        //     .withVelocityX(-y)//forwards and backwards? YES
        //     .withVelocityY(x) 
        //     .withRotationalRate(rotationOutput));

    }
    
    @Override
    public void end(boolean interrupted) {
        System.out.println("ENDED!!!!!!!!!!!!!!!!!!!!!!!!!");
        // drivetrain.setControl(m_alignRequest
        //     .withVelocityX(0)
        //     .withVelocityY(0)
        //     .withRotationalRate(0));

        drivetrain.setControl(request);
        
    }

    @Override
    public boolean isFinished() {
        return !LimelightHelpers.getTV(VisionConstants.LL_CENTER); //aimController.atGoal();
    }

    // public align withTolerance(double aimToleranceDeg, double rangeToleranceMeters) {
    //     aimController.setTolerance(Math.toRadians(aimToleranceDeg));
    //     rangeController.setTolerance(rangeToleranceMeters);
    //     return this;
    // }
}