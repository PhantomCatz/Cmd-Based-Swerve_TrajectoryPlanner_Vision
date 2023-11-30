package frc.robot.commands.DriveCmds.Trajectory;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.CatzDriveTrainSubsystem;
import frc.robot.subsystems.drivetrain.CatzDriveTrainSubsystem.DriveConstants;

// Follows a trajectory
public class TrajectoryFollowingCmd extends CommandBase{
    private final double TIMEOUT_RATIO = 2.5;
    private final double END_POS_ERROR = 0.05;
    private final double END_ROT_ERROR = 2.5; //degrees

    private final Timer timer = new Timer();
    private final HolonomicDriveController controller;
    private CatzDriveTrainSubsystem m_driveTrain = CatzDriveTrainSubsystem.getInstance();

    private final Trajectory trajectory;
    private final Rotation2d targetHeading;
    private Rotation2d initHeading;

    /**
     * @param trajectory The trajectory to follow
     * @param refHeading The goal heading for the robot to be in while in the middle of the trajectory. Takes a Pose2d parameter so that the heading may change based on external factors. 
     */
    public TrajectoryFollowingCmd(Trajectory trajectory, Rotation2d targetHeading)
    {
        this.trajectory = trajectory;
        this.targetHeading = targetHeading; // this returns the desired orientation when given the current position (the function itself is given as an argument). But most of the times, it will just give a constant desired orientation.
        // also, why is it called refheading? wouldn't something like targetOrientation be better

        controller = DriveConstants.holonomicDriveController; // see catzconstants
       addRequirements(m_driveTrain);
    }

    // reset and start timer
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        initHeading = m_driveTrain.getRotation2d();
    }

    // calculates if trajectory is finished
    @Override
    public boolean isFinished() {
        double maxTime = trajectory.getTotalTimeSeconds();
        Pose2d currentPosition = m_driveTrain.getPose();
        Pose2d dist = trajectory.sample(maxTime).poseMeters.relativeTo(currentPosition);

        double angleError = Math.abs(targetHeading.getDegrees() - currentPosition.getRotation().getDegrees());
        double posError = Math.hypot(dist.getX(), dist.getY());

        System.out.println("Angle error: " + angleError);
        System.out.println("Pos error: " + posError);
        System.out.println("Time left: " + (maxTime - timer.get()));
        return 
            timer.get() > maxTime * TIMEOUT_RATIO || 
            (
                angleError <= END_ROT_ERROR &&
                posError <= END_POS_ERROR
            );
    }

    // sets swerve modules to their target states so that the robot will follow the trajectory
    // see catzconstants
    @Override
    public void execute() {
        double currentTime = timer.get();
        Trajectory.State goal = trajectory.sample(currentTime);
        Pose2d currentPosition = m_driveTrain.getPose();
        Rotation2d targetHeadingNow = initHeading.interpolate(targetHeading, currentTime / trajectory.getTotalTimeSeconds());
        
        ChassisSpeeds adjustedSpeed = controller.calculate(currentPosition, goal, targetHeadingNow);
        SwerveModuleState[] targetModuleStates = DriveConstants.swerveDriveKinematics.toSwerveModuleStates(adjustedSpeed);
        m_driveTrain.setModuleStates(targetModuleStates);

        Logger.getInstance().recordOutput("Current Position", currentPosition);
        Logger.getInstance().recordOutput("Target Position", goal.poseMeters);
        Logger.getInstance().recordOutput("Adjusted VelX", adjustedSpeed.vxMetersPerSecond);
        Logger.getInstance().recordOutput("Adjusted VelX", adjustedSpeed.vyMetersPerSecond);
        Logger.getInstance().recordOutput("Adjusted VelW", adjustedSpeed.omegaRadiansPerSecond);
    }

    // stop all robot motion
    @Override
    public void end(boolean interrupted) {
        timer.stop();

        m_driveTrain.stopDriving();
        
        System.out.println("trajectory done");
    }
}