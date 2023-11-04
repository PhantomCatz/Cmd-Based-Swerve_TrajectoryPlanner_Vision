package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.Robot;
import frc.robot.subsystems.vision.CatzAprilTag;;


public class CatzDriveTrainSubsystem extends SubsystemBase
{
    private static CatzDriveTrainSubsystem instance;

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private static CatzSwerveModule[] swerveModules = new CatzSwerveModule[4];

    private static SwerveDrivePoseEstimator poseEstimator;
    private static ChassisSpeeds chassisSpeeds;
    private static CatzAprilTag aprilTag = CatzAprilTag.getInstance();

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

    private final double LT_FRNT_OFFSET =  0.0100; //0.073 //-0.0013; //MC ID 2
    private final double LT_BACK_OFFSET =  0.0439; //0.0431 //0.0498; //MC ID 4
    private final double RT_BACK_OFFSET =  0.2588; //0.2420 //0.2533; //MC ID 6
    private final double RT_FRNT_OFFSET =  0.0280; //0.0238 //0.0222; //MC ID 8

    private final double NOT_FIELD_RELATIVE = 0.0;


    private boolean modifyDrvPwr = false;
    

    public double dataJoystickAngle;
    public double dataJoystickPower;
    private int index;

    private double testAngleInc = 0.0;


    private CatzDriveTrainSubsystem()
    {   
        switch(CatzConstants.currentMode)
        {
        case REAL:
            gyroIO = new GyroIONavX();
        break;
        case REPLAY:
            gyroIO = new GyroIONavX() {};
        break;

        default:
            gyroIO = null; // new GyroIOSim();
        break;
        }
        
        LT_FRNT_MODULE = new CatzSwerveModule(LT_FRNT_DRIVE_ID, LT_FRNT_STEER_ID, LT_FRNT_ENC_PORT, LT_FRNT_OFFSET, false, 0);
        LT_BACK_MODULE = new CatzSwerveModule(LT_BACK_DRIVE_ID, LT_BACK_STEER_ID, LT_BACK_ENC_PORT, LT_BACK_OFFSET, false, 1);
        RT_BACK_MODULE = new CatzSwerveModule(RT_BACK_DRIVE_ID, RT_BACK_STEER_ID, RT_BACK_ENC_PORT, RT_BACK_OFFSET, false, 2);
        RT_FRNT_MODULE = new CatzSwerveModule(RT_FRNT_DRIVE_ID, RT_FRNT_STEER_ID, RT_FRNT_ENC_PORT, RT_FRNT_OFFSET, false, 3);

        swerveModules[0] = LT_FRNT_MODULE;
        swerveModules[1] = LT_BACK_MODULE;
        swerveModules[2] = RT_BACK_MODULE;
        swerveModules[3] = RT_FRNT_MODULE;

        LT_FRNT_MODULE.resetMagEnc();
        LT_BACK_MODULE.resetMagEnc();
        RT_FRNT_MODULE.resetMagEnc();
        RT_BACK_MODULE.resetMagEnc();

        poseEstimator = new SwerveDrivePoseEstimator(CatzConstants.DriveConstants.swerveDriveKinematics, 
                                                     Rotation2d.fromDegrees(0), 
                                                     getModulePositions(), 
                                                     new Pose2d(0,0,Rotation2d.fromDegrees(0)));
        
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroGyro();
            } catch (Exception e) {
            }
        }).start();
        
        //For pathplanner trajectory auton
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPosition, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                4.5, // Max module speed, in m/s
                0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            this // Reference to this subsystem to set requirements
        );
    }   
    


    @Override
    public void periodic() 
    {
        //update inputs(sensors/encoders) for code logic and advantage kit
        for(CatzSwerveModule module : swerveModules)
        {
            module.periodic();
        }
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Drive/gyroinputs ", gyroInputs);
        Pose2d aprilPose2d = null;
        
        poseEstimator.updateWithTime(Logger.getInstance().getTimestamp(), getRotation2d(), getModulePositions());

        if(aprilTag.aprilTagInView())
        {
            aprilPose2d = aprilTag.getLimelightBotPose();
            poseEstimator.addVisionMeasurement(aprilPose2d, Logger.getInstance().getTimestamp());
        }
        
        //logging
        Logger.getInstance().recordOutput("Drive/VisionPose" , aprilPose2d);
        Logger.getInstance().recordOutput("Obometry/pose", getPose());
        Logger.getInstance().recordOutput("Drive/rotationheading" , getHeading());
    }
    

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds)
    {
        SwerveModuleState[] moduleStates = CatzConstants.DriveConstants.swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) 
    {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, CatzConstants.DriveConstants.MAX_SPEED);
        swerveModules[0].setDesiredState(desiredStates[2]);
        swerveModules[1].setDesiredState(desiredStates[1]);
        swerveModules[2].setDesiredState(desiredStates[3]);
        swerveModules[3].setDesiredState(desiredStates[0]);
    }

    public void setBrakeMode() {
        for(CatzSwerveModule module : swerveModules) {
            module.setBrakeMode();
        }
    }

    public void setCoastMode(){
        for(CatzSwerveModule module : swerveModules) {
            module.setCoastMode();
        }
    }

    public void zeroGyro() {
        gyroIO.setAngleAdjustmentIO(-gyroInputs.gyroYaw);
    }

    public double getRollAngle() {
        return gyroInputs.gyroRoll;
    }

    public Pose2d getPose()
    {
        return poseEstimator.getEstimatedPosition();
    }

    public double getGyroAngle(){
        return gyroInputs.gyroAngle;
    }

    public double getHeading() {
        return Math.IEEEremainder(getGyroAngle(), 360);
    }

    public double getHeadingRadians() {
        return getHeading() * Math.PI/180;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromRadians(getHeadingRadians());
    }

    private void resetPosition(Pose2d pose)
    {
        poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    private ChassisSpeeds getChassisSpeeds()
    {
        return chassisSpeeds;
    }

    private void resetMagEncs()
    {
        for(CatzSwerveModule module : swerveModules)
        {
            module.resetMagEnc();
        }
    }

    public void resetDriveEncs()
    {
        for(CatzSwerveModule module : swerveModules)
        {
            module.resetDriveEncs();
        }
    }

    public void initializeOffsets()
    {
        gyroIO.setAngleAdjustmentIO(-gyroInputs.gyroYaw);

        for(CatzSwerveModule module : swerveModules)
        {
            module.initializeOffset();
        }
    }

    public void stopDriving(){
        for(CatzSwerveModule module : swerveModules)
        {
            module.setDrivePercent(0.0);
            module.setSteerPower(0.0);
        }
    }

    public SwerveModuleState[] getModuleStates()
    {
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        for(int i = 0; i < 4; i++)
        {
            moduleStates[i] = swerveModules[i].getModuleState();
        }
        return moduleStates;
    }

    public static SwerveModulePosition[] getModulePositions()
    {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for(int i = 0; i < 4; i++)
        {
            modulePositions[i] = swerveModules[i].getModulePosition();
        }
        return modulePositions;
    }
    
    //Singleton implementation for instatiating subssytems(Every refrence to this method should be static)
    public static CatzDriveTrainSubsystem getInstance()
    {
        if(instance == null)
        {
            instance = new CatzDriveTrainSubsystem();
        }
        return instance;
    }
}
