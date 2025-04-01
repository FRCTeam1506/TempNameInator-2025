//https://github.com/Spartronics4915/2025-Reefscape/blob/0d0a63a0bafb5364db06d54ebe7018144dd17aad/src/main/java/com/spartronics4915/frc2025/commands/autos/PositionPIDCommand.java#L76-L87

package frc.robot.commands.vision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class PIDToPose4Spark extends Command{
    
    public CommandSwerveDrivetrain drivetrain;
    public final Pose2d goalPose;

    public static SwerveRequest.ApplyRobotSpeeds rcRequest = new SwerveRequest.ApplyRobotSpeeds();

    private PPHolonomicDriveController mDriveController = new PPHolonomicDriveController(
        new PIDConstants(1, 0, 0), 
        new PIDConstants(1, 0, 0)
    );

    private final Trigger endTrigger;
    private final Trigger endTriggerDebounced;

    private PIDToPose4Spark(CommandSwerveDrivetrain drivetrain, Pose2d goalPose) {
        this.drivetrain = drivetrain;
        this.goalPose = goalPose;

        //////
        Pose2d diff = drivetrain.getState().Pose.relativeTo(goalPose);

        boolean rotation = MathUtil.isNear(
            0.0, 
            diff.getRotation().getRotations(), 
            Math.toRadians(1.5), //rotation tolerance
            0.0, 
            1.0
        );

        boolean position = diff.getTranslation().getNorm() < 0.07; //position tolernace of 0.07

        boolean speed = drivetrain.getState().Speeds.vxMetersPerSecond < 0.2;

        endTrigger = new Trigger(() -> rotation && position && speed);

        endTriggerDebounced = endTrigger.debounce(0.04);
    }

    public static Command generateCommand(CommandSwerveDrivetrain drivetrain, Pose2d goalPose, int timeout){
        return new PIDToPose4Spark(drivetrain, goalPose).withTimeout(timeout).finallyDo(() -> {
            drivetrain.setControl(rcRequest.withSpeeds(new ChassisSpeeds(0,0,0)));
        });
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        PathPlannerTrajectoryState goalState = new PathPlannerTrajectoryState();
        goalState.pose = goalPose;

        drivetrain.setControl(rcRequest.withSpeeds(
            mDriveController.calculateRobotRelativeSpeeds(
                drivetrain.getState().Pose, goalState
            )
        ));
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return endTriggerDebounced.getAsBoolean();
    }
}