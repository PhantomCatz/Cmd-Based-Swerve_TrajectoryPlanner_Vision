/*
 * Orginal swerve module class taken from Timed Atlas code
 * 
 * 
 */
package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CatzConstants;
import frc.robot.subsystems.drivetrain.CatzDriveTrainSubsystem.DriveConstants;
import frc.robot.Utils.CatzMathUtils;
import frc.robot.Utils.Conversions;
import frc.robot.subsystems.drivetrain.ModuleIOInputsAutoLogged;


public class CatzSwerveModule
{
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged   inputs = new ModuleIOInputsAutoLogged();

    private final int MOTOR_ID;


    private PIDController pid;
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

    private final double kP = 0.4; //cuz error is in tenths place so no need to mutiply kp value
    private final double kI = 0.0;
    private final double kD = 0.0;

    private double angleError = 0.0;
    private double flippedAngleError = 0.0;

    private double command;
    public boolean driveDirectionFlipped = false;

    private double wheelOffset;


    private int index;

    private double gyroAngle;


    public CatzSwerveModule(int driveMotorID, int steerMotorID, int encoderDIOChannel, double offset, int index)
    {
        this.index = index;

        switch (CatzConstants.currentMode)
        {
            case REAL:
                    io = new ModuleIOReal(driveMotorID, steerMotorID, encoderDIOChannel);
                break;
            case SIM :
                    io = new ModuleIOSim();
                break;
            default :
                    io = new ModuleIOReal(driveMotorID, steerMotorID, encoderDIOChannel) {};
                break;
        }

        pid = new PIDController(kP, kI, kD);

        wheelOffset = offset;

        //for shuffleboard
        MOTOR_ID = steerMotorID;
    }

    public void periodic() 
    {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Drive/Module " + Integer.toString(index), inputs);

        //Logging outputs
        Logger.getInstance().recordOutput("absenctorad" + Integer.toString(index) , getAbsEncRadians());

        SmartDashboard.putNumber("absenctorad" + Integer.toString(index) , getAbsEncRadians());
    }



    public void setSteerPower(double pwr)
    {
        if(CatzConstants.currentMode == CatzConstants.Mode.SIM)
        {
           io.setSteerSimPwrIO(pwr);
        }
        else
        {       
            io.setSteerPwrIO(pwr);
        }
    }

    public void setDrivePercent(double pwr)
    {
        if(CatzConstants.currentMode == CatzConstants.Mode.SIM)
        {
           io.setDriveSimPwrIO(pwr);
        }
        else
        {       
            io.setDrivePwrPercentIO(pwr);
        }
    }

    public void setDriveVelocity(double velocity)
    {
        if(CatzConstants.currentMode == CatzConstants.Mode.SIM)
        {
            
        }
        else
        {
            io.setDriveVelocityIO(velocity);
        }
    }

    public double getDrvDistanceRaw()
    {
        return inputs.driveMtrSensorPosition;
    }

    public void setCoastMode()
    {
        io.setSteerCoastModeIO();
    }

    public void setBrakeMode()
    {
        io.setSteerBrakeModeIO();
    }

    public void resetDrvDistance()
    {
        int i = 0;

        io.setDrvSensorPositionIO(0.0); //resetsensorpos
        while(Math.abs(inputs.driveMtrSensorPosition) > 1.0)
        {
            i++;
            if(i >= 3000)
            {
                resetDrvDistance();
            }
        }
    }

    public double getDrvVelocity()
    {
        return inputs.driveMtrVelocity;
    }
    
    private double getAbsEncRadians()
    {
        return (inputs.magEncoderValue - wheelOffset) * 2 * Math.PI;
    }

    public double getError()
    {
        return angleError;
    }

    public double getFlipError()
    {
        return flippedAngleError;
    }

    /*Auto Balance */
    public void reverseDrive(Boolean reverse)
    {
        io.reverseDriveIO(reverse);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param state Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState state) {

        state = CatzMathUtils.optimize(state, getCurrentRotation());
        //calculate drive pwr
        double drivePwrVelocity = Conversions.MPSToFalcon(state.speedMetersPerSecond, 
                                                          DriveConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE, 
                                                          DriveConstants.SDS_L2_GEAR_RATIO); //to set is as a gear reduction not an overdrive
        //calculate turn pwr
        double steerPIDpwr = - pid.calculate(getAbsEncRadians(), state.angle.getRadians()); 

        //set powers
        setDriveVelocity(drivePwrVelocity);// + driveFeedforward);
        setSteerPower(steerPIDpwr);// + turnFeedforward);

        //logging
        Logger.getInstance().recordOutput("Drive/current roation" + Integer.toString(index), getAbsEncRadians());
        Logger.getInstance().recordOutput("Drive/target Angle" + Integer.toString(index), state.angle.getRadians());
        Logger.getInstance().recordOutput("Drive/drive velocity" + Integer.toString(index), drivePwrVelocity);
        Logger.getInstance().recordOutput("Drive/turn power" + Integer.toString(index), steerPIDpwr);
       // Logger.getInstance().recordOutput("rotation" + Integer.toString(index), d);
    }


    public void resetMagEnc()
    {
        
    }

    public void resetDriveEncs()
    {
        io.setDrvSensorPositionIO(0.0);
    }

    public void initializeOffset()
    {
        //TBD
    }

    //inputs the rotation object as radian conversion
    public Rotation2d getCurrentRotation()
    {
        return new Rotation2d(getAbsEncRadians());
    }

    public SwerveModuleState getModuleState()
    {
        double velocity = Conversions.falconToMPS(inputs.driveMtrVelocity, DriveConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE, DriveConstants.SDS_L2_GEAR_RATIO);
        
        return new SwerveModuleState(velocity, getCurrentRotation());
    }

    public SwerveModulePosition getModulePosition()
    {
        return new SwerveModulePosition(getDriveDistanceMeters(), getCurrentRotation());
    }
    
    public double getDriveDistanceMeters()
    {
        return  ((inputs.driveMtrSensorPosition / 2048)/ DriveConstants.SDS_L2_GEAR_RATIO) * DriveConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE;
    }
}
