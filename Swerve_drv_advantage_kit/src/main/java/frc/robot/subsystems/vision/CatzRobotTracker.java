package frc.robot.subsystems.vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.CatzDriveTrainSubsystem;
import frc.robot.CatzConstants;

public class CatzRobotTracker extends CommandBase{
    // combines SwerveDrivePoseEstimator with april tags to get robot position

    private static CatzRobotTracker instance = null;

    private static final int THREAD_PERIOD_MS = 20;

    private final CatzDriveTrainSubsystem driveTrain = CatzDriveTrainSubsystem.getInstance();
    private final CatzAprilTag limelight =  CatzAprilTag.getInstance();
    
    private SwerveDrivePoseEstimator poseEstimator;

    private CatzRobotTracker()
    {
        driveTrain.resetDriveEncs();
        poseEstimator = new SwerveDrivePoseEstimator(CatzConstants.DriveConstants.swerveDriveKinematics, Rotation2d.fromDegrees(0), driveTrain.getModulePositions(), new Pose2d(0,0,Rotation2d.fromDegrees(0)));
    }

    //METHOD USED IN AUTONOMOUS CONTAINER
    public void resetPosition(Pose2d pose)
    {
        driveTrain.resetDriveEncs();
        poseEstimator.resetPosition(Rotation2d.fromDegrees(driveTrain.getGyroAngle()), driveTrain.getModulePositions(), pose);
    }
    //METHOD USED IN AUTONOMOUS CONTAINER

    // returns itself
    public static CatzRobotTracker getInstance()
    {
        if(instance == null) 
        {
            instance = new CatzRobotTracker();
        }
        return instance;
    }

    // get position
    public Pose2d getEstimatedPosition()
    {
        return poseEstimator.getEstimatedPosition();
    }

    public SwerveDrivePoseEstimator getEstimator()
    {
        return poseEstimator;
    }


    // updates poseEstimator with new measurements
    @Override
    public void execute() 
    {
        if(limelight.aprilTagInView())
        {
            poseEstimator.addVisionMeasurement(limelight.getLimelightBotPose(), Timer.getFPGATimestamp());
        }
        poseEstimator.update(Rotation2d.fromDegrees(driveTrain.getGyroAngle()), driveTrain.getModulePositions());
        System.out.println("("+poseEstimator.getEstimatedPosition().getX()+","+poseEstimator.getEstimatedPosition().getY()+")");
    }

}
