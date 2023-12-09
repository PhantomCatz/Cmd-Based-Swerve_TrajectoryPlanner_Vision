package frc.robot.Subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.Robot;
import frc.robot.CatzConstants.DriveConstants;
import frc.robot.Commands.TeleopDriveCmd;
import frc.robot.Subsystems.vision.CatzAprilTag;
import frc.robot.Utils.GeometryUtils;



public class SubsystemCatzDriveTrain extends SubsystemBase {
 //---------------------CatzDriveTrain class Definitions------------------------------------
    private static SubsystemCatzDriveTrain instance = new SubsystemCatzDriveTrain();

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private CatzSwerveModule[] m_swerveModules = new CatzSwerveModule[4];

    private static SwerveDrivePoseEstimator m_poseEstimator;
    private static CatzAprilTag m_aprilTag = CatzAprilTag.getInstance();

    public final CatzSwerveModule LT_FRNT_MODULE;
    public final CatzSwerveModule LT_BACK_MODULE;
    public final CatzSwerveModule RT_FRNT_MODULE;
    public final CatzSwerveModule RT_BACK_MODULE;

    private SubsystemCatzDriveTrain() {   

        switch(CatzConstants.currentMode) {
        case REAL: gyroIO = new GyroIONavX();
        break;
        case REPLAY: gyroIO = new GyroIONavX() {};
        break;

        default: gyroIO = null;
        break;
        }
        
        LT_FRNT_MODULE = new CatzSwerveModule(DriveConstants.LT_FRNT_DRIVE_ID, 
                                              DriveConstants.LT_FRNT_STEER_ID, 
                                              DriveConstants.LT_FRNT_ENC_PORT, 
                                              DriveConstants.LT_FRNT_OFFSET, 0);

        LT_BACK_MODULE = new CatzSwerveModule(DriveConstants.LT_BACK_DRIVE_ID, 
                                              DriveConstants.LT_BACK_STEER_ID, 
                                              DriveConstants.LT_BACK_ENC_PORT, 
                                              DriveConstants.LT_BACK_OFFSET, 1);

        RT_BACK_MODULE = new CatzSwerveModule(DriveConstants.RT_BACK_DRIVE_ID, 
                                              DriveConstants.RT_BACK_STEER_ID, 
                                              DriveConstants.RT_BACK_ENC_PORT, 
                                              DriveConstants.RT_BACK_OFFSET, 2);

        RT_FRNT_MODULE = new CatzSwerveModule(DriveConstants.RT_FRNT_DRIVE_ID, 
                                              DriveConstants.RT_FRNT_STEER_ID, 
                                              DriveConstants.RT_FRNT_ENC_PORT, 
                                              DriveConstants.RT_FRNT_OFFSET, 3);

        m_swerveModules[0] = LT_FRNT_MODULE;
        m_swerveModules[1] = LT_BACK_MODULE;
        m_swerveModules[2] = RT_BACK_MODULE;
        m_swerveModules[3] = RT_FRNT_MODULE;

        zeroGyro();

        m_poseEstimator = new SwerveDrivePoseEstimator(
            DriveConstants.swerveDriveKinematics, 
            DriveConstants.initPose.getRotation(), 
            getModulePositions(), 
            DriveConstants.initPose
        );
    }   

    @Override
    public void periodic() {
        //update inputs(sensors/encoders) for code logic and advantage kit
            LT_FRNT_MODULE.periodic();
            LT_BACK_MODULE.periodic();
            RT_BACK_MODULE.periodic();
            RT_FRNT_MODULE.periodic();
        
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Drive/gyroinputs ", gyroInputs);
        Pose2d aprilPose2d;
        
        //updated pose estimator
        m_poseEstimator.update(getRotation2d(), getModulePositions());

        //apriltag logic to possibly update pose estimator
        if(m_aprilTag.aprilTagInView()) {         
            aprilPose2d = m_aprilTag.getLimelightBotPose();
            m_poseEstimator.addVisionMeasurement(aprilPose2d, Logger.getInstance().getTimestamp());

            Logger.recordOutput("Drive/VisionPose" , aprilPose2d);
        }
        
        
        //logging
       Logger.recordOutput("Obometry/pose", getPose());
        Logger.recordOutput("Drive/rotationheading" , getHeadingRadians());
        m_aprilTag.smartDashboardAprilTag();

        SmartDashboard.putNumber("gyroAngle", getGyroAngle());
        SmartDashboard.putNumber("HeadingRad", getHeadingRadians());
    }
    
    //access method for updating drivetrain instructions
    public void driveRobot(ChassisSpeeds chassisSpeeds) {
        //apply second order kinematics to prevent swerve skew
        chassisSpeeds = correctForDynamics(chassisSpeeds);

        //Convert chassis speeds to individual module states and set module states
        SwerveModuleState[] moduleStates = DriveConstants.swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }

    //setting indivdual module states to each of the swerve modules
    public void setModuleStates(SwerveModuleState[] desiredStates) {
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

    //-------------------------------------------------DriveTrain MISC methods-------------------------------------------------
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

    public Command stopDriving(){
        return new TeleopDriveCmd(()-> 0.0, ()-> 0.0, ()-> 0.0,()->0.0, ()->false);
    }

    //gyro methods
    public Command zeroGyro() {
        return run(()-> gyroIO.resetNavXIO());
    }

    //negative due to weird coordinate system
    public double getGyroAngle() {
        return - gyroInputs.gyroAngle;
    }

    public double getRollAngle() {
        return gyroInputs.gyroRoll;
    }

    public Pose2d getPose() {
        Pose2d currentPosition = m_poseEstimator.getEstimatedPosition();
        currentPosition = new Pose2d(currentPosition.getX(), currentPosition.getY(), getRotation2d());
        return currentPosition;
    }

    //---------------------Heading Methods------------
    public double getHeading() {
        return Math.IEEEremainder(getGyroAngle(), 360);
    }

    public double getHeadingRadians() {
        return getHeading() * Math.PI/180;
    }

    //flipped due to weird coordinate system
    public Rotation2d getRotation2d() {
        return Rotation2d.fromRadians(getHeadingRadians());
    }

    public void resetPosition(Pose2d pose) {
        m_poseEstimator.resetPosition(pose.getRotation(), getModulePositions(), pose);
    }

    //---------------Enc resets---------------

    //TBD figured out where called
    public void resetDriveEncs() {
        for(CatzSwerveModule module : m_swerveModules) {
            module.resetDriveEncs();
        }
    }


    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        for(CatzSwerveModule module : m_swerveModules) {
            module.getModuleState();
        }
        return moduleStates;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for(CatzSwerveModule module : m_swerveModules) {
            module.getModuleState();
        }
        return modulePositions;
    }

    public static SubsystemCatzDriveTrain getInstance() {
        return instance;
    }
    
}
