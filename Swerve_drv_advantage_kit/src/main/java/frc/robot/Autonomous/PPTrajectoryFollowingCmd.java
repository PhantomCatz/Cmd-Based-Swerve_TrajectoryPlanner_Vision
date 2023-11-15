package frc.robot.Autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.CatzConstants;
import frc.robot.subsystems.drivetrain.CatzDriveTrainSubsystem;

import org.littletonrobotics.junction.Logger;

public class PPTrajectoryFollowingCmd extends CommandBase {
    private static final double ACCELERATION_COEFFICIENT = 0.2;

    public static final double ALLOWABLE_POSE_ERROR = 0.05;
    public static final double ALLOWABLE_ROTATION_ERROR = Math.toRadians(2);
    private final PPHolonomicDriveController controller;
    private static final CatzDriveTrainSubsystem driveTrain = CatzDriveTrainSubsystem.getInstance();

    /**
     * Timer object
     */
    private final Timer timer = new Timer();
    /**
     * Path Planner trajectory to follow
     */
    private PathPlannerTrajectory trajectory;

    private PathPlannerTrajectory.State previousState;

    /**
     * The auto balance on charge station command constructor.
     *
     * @param drivetrain The coordinator between the gyro and the swerve modules.
     * @param trajectory          The trajectory to follow.
     */
    public PPTrajectoryFollowingCmd(PathPlannerPath newPath) 
    {
        this.trajectory = new PathPlannerTrajectory(newPath, new ChassisSpeeds());
        controller = CatzConstants.DriveConstants.ppholonomicDriveController;
       addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        // Reset and begin timer
        this.timer.reset();
        this.timer.start();
        // Logger.getInstance().recordOutput("Drivetrain/Trajectory", trajectory);
        // Get initial state of path
        previousState = trajectory.getInitialState();
    }

    @Override
    public void execute() {
        double currentTime = this.timer.get();
        Pose2d currentPosition = driveTrain.getPose();
        // Determine desired state based on where the robot should be at the current time in the path
        PathPlannerTrajectory.State desiredState =
                (PathPlannerTrajectory.State) trajectory.sample(currentTime);
        // Transform state if necessary
        Rotation2d heading = desiredState.heading;
        // Calculate our target velocity based on current pose and desired state
        ChassisSpeeds chassisSpeeds = controller.calculateRobotRelativeSpeeds(currentPosition, desiredState);

        chassisSpeeds.vxMetersPerSecond +=
                desiredState.accelerationMpsSq * heading.getCos() * ACCELERATION_COEFFICIENT;
        chassisSpeeds.vyMetersPerSecond +=
                desiredState.accelerationMpsSq * heading.getSin() * ACCELERATION_COEFFICIENT;

        driveTrain.driveRobotRelative(chassisSpeeds);
        Logger.getInstance().recordOutput("Desired Auto Pose", new Pose2d(desiredState.positionMeters, desiredState.heading));
    }

    @Override
    public void end(boolean interrupted) {
        this.timer.stop(); // Stop timer
        driveTrain.driveRobotRelative(new ChassisSpeeds()); // Stop motors
    }

    @Override
    public boolean isFinished() {
        // Finish command if the total time the path takes is over
        var currentPose = driveTrain.getPose();
        var desiredPose = trajectory.getEndState().positionMeters;
        double driveX = driveTrain.getPose().getX();
        double driveY = driveTrain.getPose().getY();
        double driveRotation = driveTrain.getPose().getRotation().getRadians();

        double desiredX = trajectory.getEndState().positionMeters.getX();
        double desiredY = trajectory.getEndState().positionMeters.getY();
        double desiredRotation =
                trajectory.getEndState().positionMeters.getAngle().getRadians();

        double xError = Math.abs(desiredX - driveX);
        double yError = Math.abs(desiredY - driveY);
        double rotationError = Math.abs(desiredRotation - driveRotation);

        return (xError < ALLOWABLE_POSE_ERROR
                        && yError < ALLOWABLE_POSE_ERROR
                        && rotationError < ALLOWABLE_ROTATION_ERROR)
                || timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }
}