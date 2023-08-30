package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.Robot;


public class CatzDriveTrainSubsystem extends SubsystemBase
{
    private static CatzDriveTrainSubsystem instance;

    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private static SwerveModule[] swerveModules = new SwerveModule[4];

    public final SwerveModule RT_FRNT_MODULE;
    public final SwerveModule RT_BACK_MODULE;
    public final SwerveModule LT_FRNT_MODULE;
    public final SwerveModule LT_BACK_MODULE;

    private final int LT_FRNT_DRIVE_ID = 1;
    private final int LT_BACK_DRIVE_ID = 3;
    private final int RT_BACK_DRIVE_ID = 5;
    private final int RT_FRNT_DRIVE_ID = 7;
    
    private final int LT_FRNT_STEER_ID = 2;
    private final int LT_BACK_STEER_ID = 4;
    private final int RT_BACK_STEER_ID = 6;
    private final int RT_FRNT_STEER_ID = 8;

    private final int LT_FRNT_ENC_PORT = 9;
    private final int LT_BACK_ENC_PORT = 6;
    private final int RT_BACK_ENC_PORT = 7;
    private final int RT_FRNT_ENC_PORT = 8;

    private final double LT_FRNT_OFFSET =  0.0168; //-0.0013; //MC ID 2
    private final double LT_BACK_OFFSET =  0.0432; //0.0498; //MC ID 4
    private final double RT_BACK_OFFSET =  0.2533; //0.2533; //MC ID 6
    private final double RT_FRNT_OFFSET =  0.0226; //0.0222; //MC ID 8

    private final double NOT_FIELD_RELATIVE = 0.0;

    private double steerAngle = 0.0;
    private double drivePower = 0.0;
    private double turnPower  = 0.0;
    private double gyroAngle  = 0.0;

    private boolean modifyDrvPwr = false;
    

    public double dataJoystickAngle;
    public double dataJoystickPower;
    private int index;



    private CatzDriveTrainSubsystem()
    {   
        switch(CatzConstants.currentMode)
        {
        case REAL:
            gyroIO = new GyroIONavX();
        break;
        default:
            gyroIO = null; // new GyroIOSim();
        break;
        }
        
        LT_FRNT_MODULE = new SwerveModule(LT_FRNT_DRIVE_ID, LT_FRNT_STEER_ID, LT_FRNT_ENC_PORT, LT_FRNT_OFFSET, 0);
        LT_BACK_MODULE = new SwerveModule(LT_BACK_DRIVE_ID, LT_BACK_STEER_ID, LT_BACK_ENC_PORT, LT_BACK_OFFSET, 1);
        RT_FRNT_MODULE = new SwerveModule(RT_FRNT_DRIVE_ID, RT_FRNT_STEER_ID, RT_FRNT_ENC_PORT, RT_FRNT_OFFSET, 2);
        RT_BACK_MODULE = new SwerveModule(RT_BACK_DRIVE_ID, RT_BACK_STEER_ID, RT_BACK_ENC_PORT, RT_BACK_OFFSET, 3);

        swerveModules[0] = LT_FRNT_MODULE;
        swerveModules[2] = LT_BACK_MODULE;
        swerveModules[1] = RT_FRNT_MODULE;
        swerveModules[3] = RT_BACK_MODULE;

        LT_FRNT_MODULE.resetMagEnc();
        LT_BACK_MODULE.resetMagEnc();
        RT_FRNT_MODULE.resetMagEnc();
        RT_BACK_MODULE.resetMagEnc();

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroGyro();
            } catch (Exception e) {
            }
        }).start();
    }   
    


    @Override
    public void periodic() 
    {
        for(SwerveModule module : swerveModules)
        {
            module.periodic();
        }
        gyroIO.updateInputs(gyroInputs);
        Logger.getInstance().processInputs("Drive/gyroinputs ", gyroInputs);

    }

    public void setSteerPower(double pwr) {
        for(SwerveModule module : swerveModules) {
            module.setSteerPower(pwr);
        }
    }

    public void setDrivePower(double pwr) {
        for(SwerveModule module : swerveModules) {
            module.setDrivePower(pwr);
        }
    }

    public void setBrakeMode() {
        for(SwerveModule module : swerveModules) {
            module.setBrakeMode();
        }
    }

    public void setCoastMode(){
        for(SwerveModule module : swerveModules) {
            module.setCoastMode();
        }
    }

    private void resetMagEncs() {
        for(SwerveModule module : swerveModules) {
            module.resetMagEnc();
        }
    }

    public void resetDriveEncs() {
        for(SwerveModule module : swerveModules) {
            module.resetDriveEncs();
        }
    }

    public void initializeOffsets() {
        gyroIO.setAngleAdjustmentIO(-gyroInputs.gyroYaw);

        for(SwerveModule module : swerveModules) {
            module.initializeOffset();
        }
    }

    public void zeroGyro() {
        gyroIO.setAngleAdjustmentIO(-gyroInputs.gyroYaw);
    }

    public double getRollAngle() {
        return gyroInputs.gyroRoll;
    }
/* 
    public void lockWheels() {
        LT_FRNT_MODULE.setWheelAngle(-45.0, NOT_FIELD_RELATIVE);
        LT_BACK_MODULE.setWheelAngle(45.0, NOT_FIELD_RELATIVE);
        RT_FRNT_MODULE.setWheelAngle(-135.0, NOT_FIELD_RELATIVE);
        RT_BACK_MODULE.setWheelAngle(135.0, NOT_FIELD_RELATIVE);
    }
    */


    public Rotation2d getRotation2d()
    {
        return Rotation2d.fromDegrees(getHeading());
    }

    public double getHeading() 
    {
        return Math.IEEEremainder(getGyroAngle(), 360);
    }

    public double getGyroAngle() {
        return gyroInputs.gyroAngle;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates)
    {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, CatzConstants.DriveConstants.MAX_SPEED);
        LT_FRNT_MODULE.setDesiredState(desiredStates[0]);
        LT_BACK_MODULE.setDesiredState(desiredStates[1]);
        RT_FRNT_MODULE.setDesiredState(desiredStates[2]);
        RT_BACK_MODULE.setDesiredState(desiredStates[3]);
    }
    
    public static CatzDriveTrainSubsystem getInstance()
    {

        if(instance == null)
        {

            instance = new CatzDriveTrainSubsystem();

        }

        return instance;
    
    }
}
