//the improper way to do this, just putting the values from the limelight straight to the drivetrain.
//if it works, it works -- 1506

package frc.robot.commands.vision;

import frc.robot.LimelightHelpers;
import frc.robot.Constants.SwerveConstants;
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

public class alignRotationOnly extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric m_alignRequest;

    private final ProfiledPIDController thetaController;

    boolean isFinished = false;
    //From tunerconsts and robtocontainer.java
    private static final double MULTIPLIER = .9; //2 on test bot

    public alignRotationOnly(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        
        this.m_alignRequest = new SwerveRequest.RobotCentric().withDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1).withRotationalDeadband(0.1);

        thetaController = new ProfiledPIDController(SwerveConstants.alignKP, SwerveConstants.alignKI, SwerveConstants.alignKD, new TrapezoidProfile.Constraints(SwerveConstants.tMaxVelocity, SwerveConstants.tMaxAccel));

        thetaController.enableContinuousInput(-Math.PI, Math.PI); //Wrpa from -pi to ip

        isFinished = false;
    }

    @Override
    public void initialize() {
        isFinished = false;
        thetaController.setGoal(0); // tx=0 is centered

        thetaController.setTolerance(Math.toRadians(3));
    }

    @Override
    public void execute() {
        double theta = LimelightHelpers.getTX(VisionConstants.LL_BACK);        
    
        double rotationOutput = thetaController.calculate(Math.toRadians(theta));

        drivetrain.setControl(m_alignRequest
            .withVelocityX(0)//forwards and backwards? YES
            .withVelocityY(0) 
            .withRotationalRate(rotationOutput));

        if(Math.abs(theta) < 3 ){
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
        return thetaController.atGoal(); //aimController.atGoal();
        // return thetaController.atGoal() && drivetrain.getState().Speeds.vxMetersPerSecond < 0.1;
    }
}