package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.Robot;
import frc.robot.Utils.GeometryUtils;
import frc.robot.subsystems.vision.CatzAprilTag;;


public class CatzDriveTrainSubsystem extends SubsystemBase {
      //----------------------Catz auton Constants---------------------------
    public static final class DriveConstants {
        
        private static final double MODULE_DISTANCE_FROM_CENTER = 0.298;


        //not following the original coordinate system since the robot coordinate system is inverted
        private static final Translation2d SWERVE_LEFT_FRONT_LOCATION  = new Translation2d(MODULE_DISTANCE_FROM_CENTER, MODULE_DISTANCE_FROM_CENTER);
        private static final Translation2d SWERVE_LEFT_BACK_LOCATION   = new Translation2d(-MODULE_DISTANCE_FROM_CENTER, MODULE_DISTANCE_FROM_CENTER);
        private static final Translation2d SWERVE_RIGHT_BACK_LOCATION  = new Translation2d(-MODULE_DISTANCE_FROM_CENTER, -MODULE_DISTANCE_FROM_CENTER);
        private static final Translation2d SWERVE_RIGHT_FRONT_LOCATION = new Translation2d(MODULE_DISTANCE_FROM_CENTER, -MODULE_DISTANCE_FROM_CENTER);
        
        // calculates the orientation and speed of individual swerve modules when given the motion of the whole robot
        public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
            SWERVE_LEFT_FRONT_LOCATION,
            SWERVE_LEFT_BACK_LOCATION,
            SWERVE_RIGHT_BACK_LOCATION,
            SWERVE_RIGHT_FRONT_LOCATION
        );
        

        public static final double MAX_SPEED = 4.0; // meters per second
        public static final double MAX_ANGSPEED_RAD_PER_SEC = 6.0; // radians per second

        public static final double SDS_L1_GEAR_RATIO = 8.14;       //SDS mk4i L1 ratio reduction
        public static final double SDS_L2_GEAR_RATIO = 6.75;       //SDS mk4i L2 ratio reduction
        
        public static final double DRVTRAIN_WHEEL_DIAMETER             = 0.095;
        public static final double DRVTRAIN_WHEEL_CIRCUMFERENCE        = (Math.PI * DRVTRAIN_WHEEL_DIAMETER);

        //uses a trapezoidal velocity/time graph enforced with a PID loop
        private static ProfiledPIDController autoTurnPIDController
                = new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(MAX_ANGSPEED_RAD_PER_SEC, MAX_ANGSPEED_RAD_PER_SEC));

        static{
            autoTurnPIDController.enableContinuousInput(-Math.PI, Math.PI); //offset clamped between these two values
            autoTurnPIDController.setTolerance(Math.toRadians(0.1)); //tolerable error
        }

        // calculates target chassis motion when given current position and desired trajectory
        public static final HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
            new PIDController(0.35, 0, 0), // PID values for x offset
            new PIDController(0.35, 0, 0), // PID values for y offset
            autoTurnPIDController // PID values for orientation offset
        );

        // calculates target chassis motion when given current position and desired trajectory
        public static final PPHolonomicDriveController ppholonomicDriveController = new PPHolonomicDriveController(
        new PIDConstants(0.35, 0, 0), // PID values for x offset
        new PIDConstants(0.35, 0, 0), // PID values for rotation 
        MAX_SPEED,
        MODULE_DISTANCE_FROM_CENTER
        );
    }
 //---------------------CatzDriveTrain class Definitions------------------------------------
    private static CatzDriveTrainSubsystem instance = new CatzDriveTrainSubsystem();

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private static CatzSwerveModule[] m_swerveModules = new CatzSwerveModule[4];

    private static SwerveDrivePoseEstimator m_poseEstimator;
    private static CatzAprilTag m_aprilTag = CatzAprilTag.getInstance();

    public final CatzSwerveModule LT_FRNT_MODULE;
    public final CatzSwerveModule LT_BACK_MODULE;
    public final CatzSwerveModule RT_FRNT_MODULE;
    public final CatzSwerveModule RT_BACK_MODULE;

    private final int LT_FRNT_DRIVE_ID = 1;
    private final int LT_BACK_DRIVE_ID = 3;
    private final int RT_BACK_DRIVE_ID = 22;
    private final int RT_FRNT_DRIVE_ID = 7;
    
    private final int LT_FRNT_STEER_ID = 2;
    private final int LT_BACK_STEER_ID = 4;
    private final int RT_BACK_STEER_ID = 6;
    private final int RT_FRNT_STEER_ID = 8;

    private final int LT_FRNT_ENC_PORT = 9;
    private final int LT_BACK_ENC_PORT = 6;
    private final int RT_BACK_ENC_PORT = 7;
    private final int RT_FRNT_ENC_PORT = 8;

    private final double LT_FRNT_OFFSET = 0.007371500184287522;
    private final double LT_BACK_OFFSET = 0.04736465118411634;
    private final double RT_BACK_OFFSET = 0.2542108938552728;
    private final double RT_FRNT_OFFSET = 0.0351528633788208;

    private CatzDriveTrainSubsystem() {   
        switch(CatzConstants.currentMode) {
        case REAL: gyroIO = new GyroIONavX();
        break;
        case REPLAY: gyroIO = new GyroIONavX() {};
        break;

        default: gyroIO = null; // new GyroIOSim();
        break;
        }
        
        LT_FRNT_MODULE = new CatzSwerveModule(LT_FRNT_DRIVE_ID, LT_FRNT_STEER_ID, LT_FRNT_ENC_PORT, LT_FRNT_OFFSET, 0);
        LT_BACK_MODULE = new CatzSwerveModule(LT_BACK_DRIVE_ID, LT_BACK_STEER_ID, LT_BACK_ENC_PORT, LT_BACK_OFFSET, 1);
        RT_BACK_MODULE = new CatzSwerveModule(RT_BACK_DRIVE_ID, RT_BACK_STEER_ID, RT_BACK_ENC_PORT, RT_BACK_OFFSET, 2);
        RT_FRNT_MODULE = new CatzSwerveModule(RT_FRNT_DRIVE_ID, RT_FRNT_STEER_ID, RT_FRNT_ENC_PORT, RT_FRNT_OFFSET, 3);

        m_swerveModules[0] = LT_FRNT_MODULE;
        m_swerveModules[1] = LT_BACK_MODULE;
        m_swerveModules[2] = RT_BACK_MODULE;
        m_swerveModules[3] = RT_FRNT_MODULE;
        
        LT_FRNT_MODULE.resetMagEnc();
        LT_BACK_MODULE.resetMagEnc();
        RT_FRNT_MODULE.resetMagEnc();
        RT_BACK_MODULE.resetMagEnc();

        m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.swerveDriveKinematics, 
                                                     Rotation2d.fromDegrees(0), 
                                                     getModulePositions(), 
                                                     new Pose2d(0,0,Rotation2d.fromDegrees(0)));
    }   

    @Override
    public void periodic() {
        //update inputs(sensors/encoders) for code logic and advantage kit
        for(CatzSwerveModule module : m_swerveModules) {
            module.periodic();
        }
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Drive/gyroinputs ", gyroInputs);
        Pose2d aprilPose2d;
        
        //updated pose estimator
        m_poseEstimator.updateWithTime(Logger.getInstance().getTimestamp(), getRotation2d(), getModulePositions());

        //apriltag logic to possibly update pose estimator
        if(m_aprilTag.aprilTagInView()) {         
            aprilPose2d = m_aprilTag.getLimelightBotPose();
            m_poseEstimator.addVisionMeasurement(aprilPose2d, Logger.getInstance().getTimestamp());

            Logger.getInstance().recordOutput("Drive/VisionPose" , aprilPose2d);
        }
        
        //logging
        Logger.getInstance().recordOutput("Obometry/pose", getPose());
        Logger.getInstance().recordOutput("Drive/rotationheading" , getHeadingRadians());
        m_aprilTag.smartDashboardAprilTag();

        SmartDashboard.putNumber("gyroAngle", getGyroAngle());
        SmartDashboard.putNumber("HeadingRad", getHeadingRadians());
    }
    
    //access method for updating drivetrain instructions
    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        //apply second order kinematics to prevent swerve skew
        chassisSpeeds = correctForDynamics(chassisSpeeds);

        //Convert chassis speeds to individual module states and set module states
        SwerveModuleState[] moduleStates = DriveConstants.swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }

    //setting indivdula module states to each of the swerve modules
    private void setModuleStates(SwerveModuleState[] desiredStates) {
        //scaling down wheel speeds
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_SPEED);

        //setting module stes to each of the swerve modules
        LT_FRNT_MODULE.setDesiredState(desiredStates[0]);
        LT_BACK_MODULE.setDesiredState(desiredStates[1]);
        RT_BACK_MODULE.setDesiredState(desiredStates[2]);
        RT_FRNT_MODULE.setDesiredState(desiredStates[3]);

        //logging
        Logger.getInstance().recordOutput("module states", desiredStates);
    }

    /**
     * Correction for swerve second order dynamics issue. Borrowed from 254:
     * https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/subsystems/Drive.java#L325
     * Discussion:
     * https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
     */
    private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
        final double LOOP_TIME_S = 0.02;
        Pose2d futureRobotPose =
            new Pose2d(
                originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
                originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
                Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
        Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
        ChassisSpeeds updatedSpeeds =
            new ChassisSpeeds(
                twistForPose.dx / LOOP_TIME_S,
                twistForPose.dy / LOOP_TIME_S,
                twistForPose.dtheta / LOOP_TIME_S);
        return updatedSpeeds;
    }

    //-------------------------------------------------DriveTraing method Utils-------------------------------------------------
    public void setBrakeMode() {
        for(CatzSwerveModule module : m_swerveModules) {
            module.setBrakeMode();
        }
    }

    public void setCoastMode() {
        for(CatzSwerveModule module : m_swerveModules) {
            module.setCoastMode();
        }
    }

    public void zeroGyro() {
        gyroIO.setAngleAdjustmentIO(-gyroInputs.gyroYaw);
    }

    public double getRollAngle() {
        return gyroInputs.gyroRoll;
    }

    public Pose2d getPose() {
        Pose2d currentPosition = m_poseEstimator.getEstimatedPosition();
        currentPosition = new Pose2d(currentPosition.getX(), currentPosition.getY(), getRotation2d());
        return currentPosition;
    }

    //negative due to weird coordinate system
    public double getGyroAngle() {
        double gyroAngle = - gyroInputs.gyroAngle;
        return gyroAngle;
    }

    public double getHeading() {
        return Math.IEEEremainder(getGyroAngle(), 360);
    }

    public double getHeadingRadians() {
        return getHeading() * Math.PI/180;
    }

    //flipped due to weird coordinate system
    public Rotation2d getRotation2d() {
        return Rotation2d.fromRadians(-getHeadingRadians()); //TBD remove negative if necessary
    }

    private void resetPosition(Pose2d pose) {
        m_poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    private void resetMagEncs() {
        for(CatzSwerveModule module : m_swerveModules){
            module.resetMagEnc();
        }
    }

    public void resetDriveEncs() {
        for(CatzSwerveModule module : m_swerveModules) {
            module.resetDriveEncs();
        }
    }

    public void initializeOffsets() {
        gyroIO.setAngleAdjustmentIO(-gyroInputs.gyroYaw);

        for(CatzSwerveModule module : m_swerveModules) {
            module.initializeOffset();
        }
    }

    public void stopDriving(){
        for(CatzSwerveModule module : m_swerveModules) {
            module.setDrivePercent(0.0);
            module.setSteerPower(0.0);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        for(int i = 0; i < 4; i++) {
            moduleStates[i] = m_swerveModules[i].getModuleState();
        }
        return moduleStates;
    }

    public static SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for(int i = 0; i < 4; i++) {
            modulePositions[i] = m_swerveModules[i].getModulePosition();
        }
        return modulePositions;
    }

    public static CatzDriveTrainSubsystem getInstance() {
        return instance;
    }
    
}
