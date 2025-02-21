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

public class StopDrivetrain extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric m_alignRequest;



    public StopDrivetrain(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        
        this.m_alignRequest = new SwerveRequest.RobotCentric().withDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1).withRotationalDeadband(0.1);            
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        drivetrain.setControl(m_alignRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
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
        return Math.abs(drivetrain.getState().Speeds.vxMetersPerSecond) < 0.2; //aimController.atGoal();
        // return thetaController.atGoal() && drivetrain.getState().Speeds.vxMetersPerSecond < 0.1;
    }

    // public align withTolerance(double aimToleranceDeg, double rangeToleranceMeters) {
    //     aimController.setTolerance(Math.toRadians(aimToleranceDeg));
    //     rangeController.setTolerance(rangeToleranceMeters);
    //     return this;
    // }
}