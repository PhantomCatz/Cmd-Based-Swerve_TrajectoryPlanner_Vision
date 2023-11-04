package frc.robot.Autonomous.Trajectory;

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
import frc.robot.CatzConstants;

// Follows a trajectory
public class TrajectoryFollowingCmd extends CommandBase{
    private final double TIMEOUT_RATIO = 1.5;
    private final double END_POS_ERROR = 0.05;
    private final double END_ROT_ERROR = Math.toRadians(5);

    private final Timer timer = new Timer();
    private final HolonomicDriveController controller;
    private final CatzDriveTrainSubsystem driveTrain = CatzDriveTrainSubsystem.getInstance();

    private final Trajectory trajectory;
    private final Rotation2d targetHeading;

    /**
     * @param trajectory The trajectory to follow
     * @param refHeading The goal heading for the robot to be in while in the middle of the trajectory. Takes a Pose2d parameter so that the heading may change based on external factors. 
     */
    public TrajectoryFollowingCmd(Trajectory trajectory, Rotation2d targetHeading)
    {
        this.trajectory = trajectory;
        this.targetHeading = targetHeading; // this returns the desired orientation when given the current position (the function itself is given as an argument). But most of the times, it will just give a constant desired orientation.
        // also, why is it called refheading? wouldn't something like targetOrientation be better

        controller = CatzConstants.DriveConstants.holonomicDriveController; // see catzconstants
        addRequirements(driveTrain);
    }

    // reset and start timer
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // calculates if trajectory is finished
    @Override
    public boolean isFinished() {
        double maxTime = trajectory.getTotalTimeSeconds();
        Pose2d dist = trajectory.sample(maxTime).poseMeters.relativeTo(driveTrain.getPose());

        return 
            timer.get() > maxTime * TIMEOUT_RATIO || 
            (
                dist.getRotation().getDegrees() <= END_ROT_ERROR &&
                Math.sqrt(Math.pow(dist.getX(), 2) + Math.pow(dist.getY(), 2)) <= END_POS_ERROR
            );
    }

    // sets swerve modules to their target states so that the robot will follow the trajectory
    // see catzconstants
    @Override
    public void execute() {
        double currentTime = timer.get();
        Trajectory.State goal = trajectory.sample(currentTime);
        Pose2d currentPosition = driveTrain.getPose();
        
        //ChassisSpeeds adjustedSpeed = controller.calculate(currentPosition, goal, targetHeading);
        //adjustedSpeed.omegaRadiansPerSecond = - adjustedSpeed.omegaRadiansPerSecond;
        ChassisSpeeds adjustedSpeed = new ChassisSpeeds(4, 0, 0);
        SwerveModuleState[] targetModuleStates = CatzConstants.DriveConstants.swerveDriveKinematics.toSwerveModuleStates(adjustedSpeed);
        //driveTrain.setModuleStates(targetModuleStates);

        CatzDriveTrainSubsystem.getInstance().LT_FRNT_MODULE.setDrivePercent(0.5);
        CatzDriveTrainSubsystem.getInstance().LT_BACK_MODULE.setDrivePercent(0.5);
        CatzDriveTrainSubsystem.getInstance().RT_FRNT_MODULE.setDrivePercent(0.5);
        CatzDriveTrainSubsystem.getInstance().RT_BACK_MODULE.setDrivePercent(0.5);
    }

    // stop all robot motion
    @Override
    public void end(boolean interrupted) {
        timer.stop();

        driveTrain.stopDriving();
        
        System.out.println("trajectory done");
    }
}