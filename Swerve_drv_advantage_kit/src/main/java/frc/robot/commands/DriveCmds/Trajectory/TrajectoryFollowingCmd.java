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
import frc.robot.subsystems.drivetrain.SubsystemCatzDriveTrain;
import frc.robot.CatzConstants.DriveConstants;

// Follows a trajectory
public class TrajectoryFollowingCmd extends CommandBase{
    private final double TIMEOUT_RATIO = 25;
    private final double END_POS_ERROR = 0.05;
    private final double END_ROT_ERROR = 2.5; //degrees

    private final Timer timer = new Timer();
    private final HolonomicDriveController controller;
    private SubsystemCatzDriveTrain m_driveTrain = SubsystemCatzDriveTrain.getInstance();

    private final Trajectory trajectory;
    private final Rotation2d endOrientation;
    private Rotation2d initOrientation;

    /**
     * @param trajectory The trajectory to follow
     * @param refHeading The goal heading for the robot to be in while in the middle of the trajectory. Takes a Pose2d parameter so that the heading may change based on external factors. 
     */
    public TrajectoryFollowingCmd(Trajectory trajectory, Rotation2d endOrientation)
    {
        this.trajectory = trajectory;
        this.endOrientation = endOrientation; // this returns the desired orientation when given the current position (the function itself is given as an argument). But most of the times, it will just give a constant desired orientation.
        // also, why is it called refheading? wouldn't something like targetOrientation be better

        controller = DriveConstants.holonomicDriveController; // see catzconstants
        addRequirements(m_driveTrain);
    }

    // reset and start timer
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        initOrientation = m_driveTrain.getRotation2d();
    }

    // calculates if trajectory is finished
    @Override
    public boolean isFinished() {
        double maxTime = trajectory.getTotalTimeSeconds();
        Pose2d currentPosition = m_driveTrain.getPose();
        Pose2d dist = trajectory.sample(maxTime).poseMeters.relativeTo(currentPosition);

        double angleError = Math.abs(endOrientation.getDegrees() - currentPosition.getRotation().getDegrees());
        double posError = Math.hypot(dist.getX(), dist.getY());

        System.out.println("Time left: " + (maxTime - timer.get()));
        return 
            //timer.get() > maxTime * TIMEOUT_RATIO || 
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
        Rotation2d targetOrientation = initOrientation.interpolate(endOrientation, currentTime / trajectory.getTotalTimeSeconds());
        Pose2d currentPosition = m_driveTrain.getPose();
        
        ChassisSpeeds adjustedSpeed = controller.calculate(currentPosition, goal, targetOrientation);
        SwerveModuleState[] targetModuleStates = DriveConstants.swerveDriveKinematics.toSwerveModuleStates(adjustedSpeed);
        m_driveTrain.setModuleStates(targetModuleStates);

        System.out.println("Current Position " + currentPosition);
        System.out.println("Target Position " + goal.poseMeters);
        Logger.getInstance().recordOutput("Current Position", currentPosition);
        Logger.getInstance().recordOutput("Target Position", goal.poseMeters);
        Logger.getInstance().recordOutput("Adjusted VelX", adjustedSpeed.vxMetersPerSecond);
        Logger.getInstance().recordOutput("Adjusted VelX", adjustedSpeed.vyMetersPerSecond);
        Logger.getInstance().recordOutput("Adjusted VelW", adjustedSpeed.omegaRadiansPerSecond);

        // for debugging
        // System.out.println(m_driveTrain.getPose());
        // m_driveTrain.LT_BACK_MODULE.setDesiredState(new SwerveModuleState(1.0, new Rotation2d(0.0)));
        // m_driveTrain.RT_BACK_MODULE.setDesiredState(new SwerveModuleState(1.0, new Rotation2d(0.0)));
        // m_driveTrain.LT_FRNT_MODULE.setDesiredState(new SwerveModuleState(1.0, new Rotation2d(0.0)));
        // m_driveTrain.RT_FRNT_MODULE.setDesiredState(new SwerveModuleState(1.0, new Rotation2d(0.0)));
    }

    // stop all robot motion
    @Override
    public void end(boolean interrupted) {
        timer.stop();

        m_driveTrain.stopDriving();
        
        System.out.println("trajectory done");
    }
}