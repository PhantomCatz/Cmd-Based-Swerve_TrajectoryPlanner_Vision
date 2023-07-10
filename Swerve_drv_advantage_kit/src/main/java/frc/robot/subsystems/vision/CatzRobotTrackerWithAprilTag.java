package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public class CatzRobotTrackerWithAprilTag {
    private static CatzRobotTrackerWithAprilTag instance = null;

    private final double METER_TO_INCH = 39.37;
    //private final double TICK_TO_METER = 0.000007349537*Math.PI; //L2
    private final double TICK_TO_METER = 0.0000060945178*Math.PI; //L1
    private final double MODEL_TO_CENTER_DIS = 0.2984;
    private final double INVALID_POSE_ENTY = 999.0;
    private final double DEFAULT_POSE_ENTY = 0.0;

    private final int POS_X_INDEX = 0;
    private final int POS_Y_INDEX = 1;
    private final int POS_Z_INDEX = 2;
    private final int ROT_X_INDEX = 3;
    private final int ROT_Y_INDEX = 4;
    private final int ROT_Z_INDEX = 5;

    private final Translation2d LF_WHEEL_KIN = new Translation2d(MODEL_TO_CENTER_DIS, MODEL_TO_CENTER_DIS);
    private final Translation2d LB_WHEEL_KIN = new Translation2d(-MODEL_TO_CENTER_DIS, MODEL_TO_CENTER_DIS);
    private final Translation2d RF_WHEEL_KIN = new Translation2d(MODEL_TO_CENTER_DIS, -MODEL_TO_CENTER_DIS);
    private final Translation2d RB_WHEEL_KIN = new Translation2d(-MODEL_TO_CENTER_DIS, -MODEL_TO_CENTER_DIS);
    private final SwerveDriveKinematics SWERV_KIN = new SwerveDriveKinematics(LF_WHEEL_KIN,LB_WHEEL_KIN,RF_WHEEL_KIN,RB_WHEEL_KIN);

    private final double[] INVALIDARRAY = {INVALID_POSE_ENTY,INVALID_POSE_ENTY,INVALID_POSE_ENTY,INVALID_POSE_ENTY,INVALID_POSE_ENTY,INVALID_POSE_ENTY};
    private double[] botPose = INVALIDARRAY;

    private SwerveDrivePoseEstimator poseEstimator;


    private static final int THREAD_PERIOD_MS = 20;
    //Start the Pos Esimator with initial Pos and Swerve model status
    private CatzRobotTrackerWithAprilTag(double initPx, double initPy, double initRz, double LFDis, double LFAng, double LBDis, double LBAng, double RFDis, double RFAng, double RBDis, double RBAng)
    {

        SwerveModulePosition[] initModelPos = {new SwerveModulePosition(LFDis*TICK_TO_METER, Rotation2d.fromDegrees(LFAng)) , new SwerveModulePosition(LBDis*TICK_TO_METER, Rotation2d.fromDegrees(LBAng)) , new SwerveModulePosition(RFDis*TICK_TO_METER, Rotation2d.fromDegrees(RFAng)) ,new SwerveModulePosition(RBDis*TICK_TO_METER, Rotation2d.fromDegrees(RBAng))};
        poseEstimator = new SwerveDrivePoseEstimator(SWERV_KIN, Rotation2d.fromDegrees(initRz), initModelPos, new Pose2d(initPx/METER_TO_INCH, initPy/METER_TO_INCH, Rotation2d.fromDegrees(initRz)));
        botPose[POS_X_INDEX] = initPx;
        botPose[POS_Y_INDEX] = initPy;
        botPose[POS_Z_INDEX] = DEFAULT_POSE_ENTY;
        botPose[ROT_X_INDEX] = DEFAULT_POSE_ENTY;
        botPose[ROT_Y_INDEX] = DEFAULT_POSE_ENTY;
        botPose[ROT_Z_INDEX] = initPy;
    }

    //update the Pos Esimator with apriltag botpos and FPGATimestamp
    public void updateEstBot(double timestamp, double[] AprilTagBotPos)
    {
        if(AprilTagBotPos[0] != INVALID_POSE_ENTY)
        {
            poseEstimator.addVisionMeasurement(new Pose2d(AprilTagBotPos[POS_X_INDEX]/METER_TO_INCH, AprilTagBotPos[POS_Y_INDEX]/METER_TO_INCH, Rotation2d.fromDegrees(AprilTagBotPos[ROT_Z_INDEX])), timestamp);
            botPose[POS_X_INDEX] = -poseEstimator.getEstimatedPosition().getX()*METER_TO_INCH;
            botPose[POS_Y_INDEX] = -poseEstimator.getEstimatedPosition().getY()*METER_TO_INCH;
            botPose[POS_Z_INDEX] = DEFAULT_POSE_ENTY;
            botPose[ROT_X_INDEX] = DEFAULT_POSE_ENTY;
            botPose[ROT_Y_INDEX] = DEFAULT_POSE_ENTY;
            botPose[ROT_Z_INDEX] = poseEstimator.getEstimatedPosition().getRotation().getDegrees();
        }
    }

    //update the Pos Esimator with gyro angle, current model status and FPGATimestamp;
    public void updateEstBotPos(double timestamp, double BotRz, double LFDis, double LFAng, double LBDis, double LBAng, double RFDis, double RFAng, double RBDis, double RBAng)
    {
        SwerveModulePosition[] modelPos = {new SwerveModulePosition(LFDis*TICK_TO_METER, Rotation2d.fromDegrees(LFAng)) , new SwerveModulePosition(LBDis*TICK_TO_METER, Rotation2d.fromDegrees(LBAng)) , new SwerveModulePosition(RFDis*TICK_TO_METER, Rotation2d.fromDegrees(RFAng)) ,new SwerveModulePosition(RBDis*TICK_TO_METER, Rotation2d.fromDegrees(RBAng))};
        poseEstimator.updateWithTime(timestamp,Rotation2d.fromDegrees(BotRz), modelPos);
        botPose[POS_X_INDEX] = -poseEstimator.getEstimatedPosition().getX()*METER_TO_INCH;
        botPose[POS_Y_INDEX] = -poseEstimator.getEstimatedPosition().getY()*METER_TO_INCH;
        botPose[POS_Z_INDEX] = DEFAULT_POSE_ENTY;
        botPose[ROT_X_INDEX] = DEFAULT_POSE_ENTY;
        botPose[ROT_Y_INDEX] = DEFAULT_POSE_ENTY;
        botPose[ROT_Z_INDEX] = poseEstimator.getEstimatedPosition().getRotation().getDegrees();

    }

    //reset Pos Esimator to new pos
    public void resetEstBotPos(double newPx, double newPy, double newRz,double LFDis, double LFAng, double LBDis, double LBAng, double RFDis, double RFAng, double RBDis, double RBAng)
    {
        SwerveModulePosition[] newModelPos = {new SwerveModulePosition(LFDis*TICK_TO_METER, Rotation2d.fromDegrees(LFAng)) , new SwerveModulePosition(LBDis*TICK_TO_METER, Rotation2d.fromDegrees(LBAng)) , new SwerveModulePosition(RFDis*TICK_TO_METER, Rotation2d.fromDegrees(RFAng)) ,new SwerveModulePosition(RBDis*TICK_TO_METER, Rotation2d.fromDegrees(RBAng))};
        poseEstimator.resetPosition(Rotation2d.fromDegrees(newRz), newModelPos, new Pose2d(newPx/METER_TO_INCH, newPy/METER_TO_INCH, Rotation2d.fromDegrees(newRz)));
        botPose[POS_X_INDEX] = -poseEstimator.getEstimatedPosition().getX()*METER_TO_INCH;
        botPose[POS_Y_INDEX] = -poseEstimator.getEstimatedPosition().getY()*METER_TO_INCH;
        botPose[POS_Z_INDEX] = DEFAULT_POSE_ENTY;
        botPose[ROT_X_INDEX] = DEFAULT_POSE_ENTY;
        botPose[ROT_Y_INDEX] = DEFAULT_POSE_ENTY;
        botPose[ROT_Z_INDEX] = poseEstimator.getEstimatedPosition().getRotation().getDegrees();
    }

    //get the Pos Estimator estimated current botpos
    public double[] getEstBotPos()
    {
        return botPose;
    }

    public static CatzRobotTrackerWithAprilTag getInstance()
    {
        if(instance == null) instance = new CatzRobotTrackerWithAprilTag(THREAD_PERIOD_MS, THREAD_PERIOD_MS, THREAD_PERIOD_MS, THREAD_PERIOD_MS, THREAD_PERIOD_MS, THREAD_PERIOD_MS, THREAD_PERIOD_MS, THREAD_PERIOD_MS, THREAD_PERIOD_MS, THREAD_PERIOD_MS, THREAD_PERIOD_MS);

        return instance;
    }

    
    public void periodic() {
        
        
    }
    

}