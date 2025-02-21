//the improper way to do this, just putting the values from the limelight straight to the drivetrain.
//if it works, it works -- 1506

package frc.robot.commands.vision;

import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class align3d extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric m_alignRequest;

    private final ProfiledPIDController thetaController;
    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;

    boolean isFinished = false;
    //From tunerconsts and robtocontainer.java
    private static final double MAX_AIM_VELOCITY = 0.5*Math.PI; // radd/s
    private static final double MAX_AIM_ACCELERATION = Math.PI / 2; // rad/s^2
    private static final double MAX_RANGE_VELOCITY = 3.5; // m/s  //2.0
    private static final double MAX_RANGE_ACCELERATION = 2.5; // m/2^s //1.5
  
    // Todo - Tune later
    private static final double AIM_P = 5; //Proprotinal  //5
    private static final double AIM_I = 0.01; //Gradual corretction  //0.01
    private static final double AIM_D = 0.2;//0.05; //Smooth oscilattions  //0.2

    private static final double HOLD_DISTANCE = 100; // OG 0.40 // y hold distance
    
    private static final double MULTIPLIER = .9; //2 on test bot

    public align3d(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        
        this.m_alignRequest = new SwerveRequest.RobotCentric().withDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1).withRotationalDeadband(0.1);

        thetaController = new ProfiledPIDController(AIM_P, AIM_I, AIM_D, new TrapezoidProfile.Constraints(MAX_AIM_VELOCITY, MAX_AIM_ACCELERATION));
        xController = new ProfiledPIDController(AIM_P, AIM_I, AIM_D, new TrapezoidProfile.Constraints(MAX_RANGE_VELOCITY, MAX_RANGE_ACCELERATION));
        yController = new ProfiledPIDController(AIM_P, AIM_I, AIM_D, new TrapezoidProfile.Constraints(MAX_RANGE_VELOCITY, MAX_RANGE_ACCELERATION));

        thetaController.enableContinuousInput(-Math.PI, Math.PI); //Wrpa from -pi to ip
            
    }

    @Override
    public void initialize() {
        // LimelightHelpers.setPipelineIndex(VisionConstants.LL_BACK, 0);
        
        // aimController.reset(LimelightHelpers.getTX(Constants.LIMELIGHT_NAME));
        // rangeController.reset(LimelightHelpers.getDistance(this.pipelineID,Constants.REEF_APRILTAG_HEIGHT)); //Init dist
        
        //thetaController.setGoal(0); // tx=0 is centered
        thetaController.setGoal(0.4); // tx=0 is centered
        xController.setGoal(0); //0
        yController.setGoal(HOLD_DISTANCE);

        thetaController.setTolerance(Math.toRadians(5));
    }

    @Override
    public void execute() {
        double theta = LimelightHelpers.getTX(VisionConstants.LL_BACK);        
    
        
        double x = LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_BACK)[0];
        double y = LimelightHelpers.getCameraPose_TargetSpace(VisionConstants.LL_BACK)[2];

        System.out.println("x: " + x + ", y: " + y);

        // double currentDistance = Constants.pythagoreanTheorem(llvalue_x, llvalue_y);

        double rotationOutput = thetaController.calculate(Math.toRadians(theta));
        double xOutput = xController.calculate(x);
        double yOutput = yController.calculate(y);

        // Translation2d translation = new Translation2d(rangeOutput, 0);

        drivetrain.setControl(m_alignRequest
            .withVelocityX(-y * MULTIPLIER * -1)//forwards and backwards? YES
            .withVelocityY(x * MULTIPLIER * -1) 
            .withRotationalRate(rotationOutput));

        if(Math.abs(y) < 0.46 ){
            isFinished = true;
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(m_alignRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
        return isFinished; //aimController.atGoal();
        // return thetaController.atGoal() && drivetrain.getState().Speeds.vxMetersPerSecond < 0.1;
    }
}