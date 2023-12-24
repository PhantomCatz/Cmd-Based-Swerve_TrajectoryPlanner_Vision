package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.DriveConstants;
import frc.robot.Utils.GeometryUtils;
import frc.robot.commands.DriveCmds.TeleopDriveCmd;
import frc.robot.subsystems.vision.SubsystemCatzVision;;

// Drive train subsystem for swerve drive implementation
public class SubsystemCatzDrivetrain extends SubsystemBase {

    // Singleton instance of the CatzDriveTrainSubsystem
    private static SubsystemCatzDrivetrain instance = new SubsystemCatzDrivetrain();

    // Gyro input/output interface
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    //Vision instatiation
    private final SubsystemCatzVision vision = SubsystemCatzVision.getInstance();

    // Array of swerve modules representing each wheel in the drive train
    private CatzSwerveModule[] m_swerveModules = new CatzSwerveModule[4];

    // Swerve drive pose estimator for tracking robot pose
    private static SwerveDrivePoseEstimator m_poseEstimator;
    private static SubsystemCatzVision m_aprilTag = SubsystemCatzVision.getInstance();

    // Swerve modules representing each corner of the robot
    public final CatzSwerveModule LT_FRNT_MODULE;
    public final CatzSwerveModule LT_BACK_MODULE;
    public final CatzSwerveModule RT_FRNT_MODULE;
    public final CatzSwerveModule RT_BACK_MODULE;

    // Private constructor for the singleton instance
    private SubsystemCatzDrivetrain() {
        // Determine gyro input/output based on the robot mode
        switch (CatzConstants.currentMode) {
            case REAL:
                gyroIO = new GyroIONavX();
                break;
            case REPLAY:
                gyroIO = new GyroIONavX() {};
                break;
            default:
                gyroIO = null;
                break;
        }

        // Create swerve modules for each corner of the robot
        LT_FRNT_MODULE = new CatzSwerveModule(DriveConstants.LT_FRNT_DRIVE_ID, DriveConstants.LT_FRNT_STEER_ID,
                DriveConstants.LT_FRNT_ENC_PORT, DriveConstants.LT_FRNT_OFFSET, 0);

        LT_BACK_MODULE = new CatzSwerveModule(DriveConstants.LT_BACK_DRIVE_ID, DriveConstants.LT_BACK_STEER_ID,
                DriveConstants.LT_BACK_ENC_PORT, DriveConstants.LT_BACK_OFFSET, 1);

        RT_BACK_MODULE = new CatzSwerveModule(DriveConstants.RT_BACK_DRIVE_ID, DriveConstants.RT_BACK_STEER_ID,
                DriveConstants.RT_BACK_ENC_PORT, DriveConstants.RT_BACK_OFFSET, 2);

        RT_FRNT_MODULE = new CatzSwerveModule(DriveConstants.RT_FRNT_DRIVE_ID, DriveConstants.RT_FRNT_STEER_ID,
                DriveConstants.RT_FRNT_ENC_PORT, DriveConstants.RT_FRNT_OFFSET, 3);

        // Assign swerve modules to the array for easier access
        m_swerveModules[0] = LT_FRNT_MODULE;
        m_swerveModules[1] = LT_BACK_MODULE;
        m_swerveModules[2] = RT_BACK_MODULE;
        m_swerveModules[3] = RT_FRNT_MODULE;

        // Zero the gyro and reset drive encoders on initialization
        zeroGyro().execute();
        resetDriveEncs();

        // Initialize the swerve drive pose estimator
        m_poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.swerveDriveKinematics,
                DriveConstants.initPose.getRotation(), getModulePositions(), DriveConstants.initPose);
    }

    // Periodic update method for the drive train subsystem
    @Override
    public void periodic() {
        // Update inputs (sensors/encoders) for code logic and advantage kit
        LT_FRNT_MODULE.periodic();
        LT_BACK_MODULE.periodic();
        RT_BACK_MODULE.periodic();
        RT_FRNT_MODULE.periodic();

        // Update gyro inputs and log them
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Drive/gyroinputs ", gyroInputs);

        // Update pose estimator
        m_poseEstimator.update(getRotation2d(), getModulePositions());

        // AprilTag logic to possibly update pose estimator //tbd should we get a option to enable vision estimates or not
        for (int i = 0; i < vision.getVisionOdometry().size(); i++) {
            m_poseEstimator.addVisionMeasurement(
                    vision.getVisionOdometry().get(i).getPose(),
                    vision.getVisionOdometry().get(i).getTimestamp(),
                    VecBuilder.fill(
                            vision.getMinDistance(i) * DriveConstants.ESTIMATION_COEFFICIENT,
                            vision.getMinDistance(i) * DriveConstants.ESTIMATION_COEFFICIENT,
                            5.0));
        }

        Logger.getInstance().recordOutput("Drive/Pose", getPose()); // Logging the pose data
        Logger.getInstance().recordOutput("Drive/EstimatedPose", m_poseEstimator.getEstimatedPosition());

        // Logging
        Logger.getInstance().recordOutput("Obometry/pose", getPose());

        // Update SmartDashboard with the gyro angle
        SmartDashboard.putNumber("gyroAngle", getGyroAngle());
    }

    // Access method for updating drivetrain instructions
    public void driveRobot(ChassisSpeeds chassisSpeeds) {
        // Apply second-order kinematics to prevent swerve skew
        chassisSpeeds = correctForDynamics(chassisSpeeds);

        // Convert chassis speeds to individual module states and set module states
        SwerveModuleState[] moduleStates = DriveConstants.swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }

    // Set individual module states to each of the swerve modules
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        // Scale down wheel speeds
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_SPEED_DESATURATION);

        // Set module states to each of the swerve modules
        LT_FRNT_MODULE.setDesiredState(desiredStates[0]);
        LT_BACK_MODULE.setDesiredState(desiredStates[1]);
        RT_BACK_MODULE.setDesiredState(desiredStates[2]);
        RT_FRNT_MODULE.setDesiredState(desiredStates[3]);

        // Logging
        Logger.getInstance().recordOutput("module states", desiredStates);
    }

    /**
     * Correction for swerve second-order dynamics issue. Borrowed from 254:
     * https://github.com/Team254/FRC-2022-Public/blob/main/src/main/java/com/team254/frc2022/subsystems/Drive.java#L325
     * Discussion:
     * https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964
     */
    private static ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
        final double LOOP_TIME_S = 0.02;
        Pose2d futureRobotPose = new Pose2d(originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
                originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
                Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
        Twist2d twistForPose = GeometryUtils.log(futureRobotPose);
        ChassisSpeeds updatedSpeeds = new ChassisSpeeds(twistForPose.dx / LOOP_TIME_S, twistForPose.dy / LOOP_TIME_S,
                twistForPose.dtheta / LOOP_TIME_S);
        return updatedSpeeds;
    }

    //--------------------------------------------------DriveTrain MISC methods-------------------------------------------------

    // Set brake mode for all swerve modules
    public void setBrakeMode() {
        for (CatzSwerveModule module : m_swerveModules) {
            module.setBrakeMode();
        }
    }

    // Set coast mode for all swerve modules
    public void setCoastMode() {
        for (CatzSwerveModule module : m_swerveModules) {
            module.setCoastMode();
        }
    }

    // Create a command to stop driving
    public Command stopDriving() {
        return new TeleopDriveCmd(() -> 0.0, () -> 0.0, () -> 0.0, () -> 0.0, () -> false);
    }

    //----------------------------------------------Gyro methods----------------------------------------------

    // Create a command to zero the gyro
    public Command zeroGyro() {
        return run(() -> gyroIO.resetNavXIO());
    }

    // Get the gyro angle (negative due to the weird coordinate system)
    public double getGyroAngle() {
        return -gyroInputs.gyroAngle;
    }

    // Get the roll angle of the gyro
    public double getRollAngle() {
        return gyroInputs.gyroRoll;
    }

    // Get the current pose of the robot
    public Pose2d getPose() {
        Pose2d currentPosition = m_poseEstimator.getEstimatedPosition();
        currentPosition = new Pose2d(currentPosition.getX(), currentPosition.getY(), getRotation2d());
        return currentPosition;
    }

    //---------------------Heading Methods----------------

    // Get the heading of the robot
    public double getHeading() {
        return Math.IEEEremainder(getGyroAngle(), 360);
    }

    // Get the heading of the robot in radians
    public double getHeadingRadians() {
        return getHeading() * Math.PI / 180;
    }

    // Get the Rotation2d object based on the gyro angle
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getGyroAngle());
    }

    // Reset the position of the robot with a given pose
    public void resetPosition(Pose2d pose) {
        m_poseEstimator.resetPosition(pose.getRotation(), getModulePositions(), pose);
    }

    //---------------------Enc resets---------------------

    // Reset drive encoders for all swerve modules
    public void resetDriveEncs() {
        for (CatzSwerveModule module : m_swerveModules) {
            module.resetDriveEncs();
        }
    }

    // Reset gyro and position for autonomous mode
    public void resetForAutonomous() {
        zeroGyro().execute();
        resetPosition(DriveConstants.initPose);
    }

    // Get an array of swerve module states
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        for (int i = 0; i < m_swerveModules.length; i++) {
            moduleStates[i] = m_swerveModules[i].getModuleState();
        }
        return moduleStates;
    }

    // Get an array of swerve module positions
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < m_swerveModules.length; i++) {
            modulePositions[i] = m_swerveModules[i].getModulePosition();
        }
        return modulePositions;
    }

    // Get the singleton instance of the CatzDriveTrainSubsystem
    public static SubsystemCatzDrivetrain getInstance() {
        return instance;
    }
}