package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.Robot;
import frc.robot.subsystems.vision.CatzRobotTracker;

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
        gyroIO = new GyroIOSim();
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
    }   
    


    @Override
    public void periodic() 
    {
        for(SwerveModule module : swerveModules)
        {
            module.periodic();
        }

        /* 
            // Update odometry
    SwerveModuleState[] measuredStatesDiff = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      measuredStatesDiff[i] = new SwerveModuleState(
          (moduleInputs[i].drivePositionRad - lastModulePositionsRad[i])
              * wheelRadius,
          turnPositions[i]);
      lastModulePositionsRad[i] = moduleInputs[i].drivePositionRad;
    }
    ChassisSpeeds chassisStateDiff =
        kinematics.toChassisSpeeds(measuredStatesDiff);
    if (gyroInputs.connected) { // Use gyro for angular change when connected
      odometryPose =
          odometryPose.exp(new Twist2d(chassisStateDiff.vxMetersPerSecond,
              chassisStateDiff.vyMetersPerSecond,
              gyroInputs.positionRad - lastGyroPosRad));
    } else { // Fall back to using angular velocity (disconnected or sim)
      odometryPose =
          odometryPose.exp(new Twist2d(chassisStateDiff.vxMetersPerSecond,
              chassisStateDiff.vyMetersPerSecond,
              chassisStateDiff.omegaRadiansPerSecond));
    }
    lastGyroPosRad = gyroInputs.positionRad;
*/
    }

    public void cmdProcSwerve(double leftJoyX, double leftJoyY, double rightJoyX, double pwrMode)
    {
        steerAngle = calcJoystickAngle(leftJoyX, leftJoyY);
        drivePower = calcJoystickPower(leftJoyX, leftJoyY);
        turnPower  = rightJoyX;
        
        gyroAngle  = getGyroAngle();

        
        if(pwrMode > 0.9)
        {
            modifyDrvPwr = true;
        }
        else
        {
            modifyDrvPwr = false;
        }

        if(drivePower >= 0.1)
        {
            
            if(modifyDrvPwr == true)
            {
                drivePower = drivePower * 0.5;
            }
            

            if(Math.abs(turnPower) >= 0.1)
            {
                if(modifyDrvPwr == true)
                {
                    turnPower = turnPower * 0.5;
                }
                translateTurn(steerAngle, drivePower, turnPower, gyroAngle);
                }
               else
            {
                drive(steerAngle, drivePower, gyroAngle);
            }

        }
        else if(Math.abs(turnPower) >= 0.1)
        {
            if(modifyDrvPwr == true)
            {
                turnPower = turnPower * 0.5;
            }
            
            rotateInPlace(turnPower);
            
        }
        else
        {
            setSteerPower(0.0);
            setDrivePower(0.0);
        }
    }

    public void drive(double joystickAngle, double joystickPower, double gyroAngle)
    {

        for(SwerveModule module : swerveModules)
        {
            module.setWheelAngle(joystickPower, gyroAngle);
        }

        setDrivePower(joystickPower);

        dataJoystickAngle = joystickAngle;
        dataJoystickPower = joystickPower;
    }

    public void rotateInPlace(double pwr)
    {
        pwr *= 0.8; //reducing turn in place pwer
        LT_FRNT_MODULE.setWheelAngle(-45.0, NOT_FIELD_RELATIVE);
        LT_BACK_MODULE.setWheelAngle(45.0, NOT_FIELD_RELATIVE);
        RT_FRNT_MODULE.setWheelAngle(-135.0, NOT_FIELD_RELATIVE);
        RT_BACK_MODULE.setWheelAngle(135.0, NOT_FIELD_RELATIVE);

        for(SwerveModule module : swerveModules)
        {
            module.setDrivePower(pwr);
        }
    }

    public void translateTurn(double joystickAngle, double translatePower, double turnPercentage, double gyroAngle)
    {
        //how far wheels turn determined by how far joystick is pushed (max of 45 degrees)
        double turnAngle = turnPercentage * -45.0;

        if(Math.abs(closestAngle(joystickAngle, 0.0 - gyroAngle)) <= 45.0)
        {
            // if directed towards front of robot
            index = 1;
            LT_FRNT_MODULE.setWheelAngle(joystickAngle + turnAngle, gyroAngle);
            RT_FRNT_MODULE.setWheelAngle(joystickAngle + turnAngle, gyroAngle);

            LT_BACK_MODULE.setWheelAngle(joystickAngle - turnAngle, gyroAngle);
            RT_BACK_MODULE.setWheelAngle(joystickAngle - turnAngle, gyroAngle);
        }
        else if(Math.abs(closestAngle(joystickAngle, 90.0 - gyroAngle)) < 45.0)
        {
            // if directed towards left of robot
            index = 2;
            LT_FRNT_MODULE.setWheelAngle(joystickAngle + turnAngle, gyroAngle);
            LT_BACK_MODULE.setWheelAngle(joystickAngle + turnAngle, gyroAngle);

            RT_FRNT_MODULE.setWheelAngle(joystickAngle - turnAngle, gyroAngle);
            RT_BACK_MODULE.setWheelAngle(joystickAngle - turnAngle, gyroAngle);
        }
        else if(Math.abs(closestAngle(joystickAngle, 180.0 - gyroAngle)) <= 45.0)
        {
            // if directed towards back of robot
            index = 3;
            LT_BACK_MODULE.setWheelAngle(joystickAngle + turnAngle, gyroAngle);
            RT_BACK_MODULE.setWheelAngle(joystickAngle + turnAngle, gyroAngle);

            LT_FRNT_MODULE.setWheelAngle(joystickAngle - turnAngle, gyroAngle);
            RT_FRNT_MODULE.setWheelAngle(joystickAngle - turnAngle, gyroAngle);
        }
        else if(Math.abs(closestAngle(joystickAngle, -90.0 - gyroAngle)) < 45.0)
        {
            // if directed towards right of robot
            index = 4;
            RT_FRNT_MODULE.setWheelAngle(joystickAngle + turnAngle, gyroAngle);
            RT_BACK_MODULE.setWheelAngle(joystickAngle + turnAngle, gyroAngle);

            LT_FRNT_MODULE.setWheelAngle(joystickAngle - turnAngle, gyroAngle);
            LT_BACK_MODULE.setWheelAngle(joystickAngle - turnAngle, gyroAngle);
        }

        LT_FRNT_MODULE.setDrivePower(translatePower);
        LT_BACK_MODULE.setDrivePower(translatePower);
        RT_FRNT_MODULE.setDrivePower(translatePower);
        RT_BACK_MODULE.setDrivePower(translatePower);
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


    public double getGyroAngle() {
        return gyroInputs.gyroAngle;
    }

    public void autoDrive(double power) {
        LT_FRNT_MODULE.setWheelAngle(0, 0);
        LT_BACK_MODULE.setWheelAngle(0, 0);
        RT_FRNT_MODULE.setWheelAngle(0, 0);
        RT_BACK_MODULE.setWheelAngle(0, 0);

        setDrivePower(power);
    }

    public void lockWheels() {
        LT_FRNT_MODULE.setWheelAngle(-45.0, NOT_FIELD_RELATIVE);
        LT_BACK_MODULE.setWheelAngle(45.0, NOT_FIELD_RELATIVE);
        RT_FRNT_MODULE.setWheelAngle(-135.0, NOT_FIELD_RELATIVE);
        RT_BACK_MODULE.setWheelAngle(135.0, NOT_FIELD_RELATIVE);
    }

    public static CatzDriveTrainSubsystem getInstance()
    {

        if(instance == null)
        {

            instance = new CatzDriveTrainSubsystem();

        }

        return instance;
    
    }

    public SwerveModulePosition[] getModulePositions()
    {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

        for(int i = 0; i < 4; i++)
        {
            modulePositions[i] = swerveModules[i].getModulePosition();
        }

        return modulePositions;
    }

    public double calcJoystickAngle(double xJoy, double yJoy)
    {
        double angle = Math.atan(Math.abs(xJoy) / Math.abs(yJoy));
        angle *= (180 / Math.PI);

        if(yJoy <= 0)   //joystick pointed up
        {
            if(xJoy < 0)    //joystick pointed left
            {
              //no change
            }
            if(xJoy >= 0)   //joystick pointed right
            {
              angle = -angle;
            }
        }
        else    //joystick pointed down
        {
            if(xJoy < 0)    //joystick pointed left
            {
              angle = 180 - angle;
            }
            if(xJoy >= 0)   //joystick pointed right
            {
              angle = -180 + angle;
            }
        }
      return angle;
    }

    public double calcJoystickPower(double xJoy, double yJoy)
    {
      return (Math.sqrt(Math.pow(xJoy, 2) + Math.pow(yJoy, 2)));
    }

    public double closestAngle(double startAngle, double targetAngle)
    {
        // get direction
        double error = targetAngle % 360.0 - startAngle % 360.0;

        // convert from -360 to 360 to -180 to 180
        if (Math.abs(error) > 180.0)
        {
            error = -(Math.signum(error) * 360.0) + error;
            //closest angle shouldn't be more than 180 degrees. If it is, use other direction
            if(error > 180.0)
            {
                error -= 360;
            }
        }

        return error;
    }
}
