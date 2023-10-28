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
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.DriveConstants;
import frc.robot.Utils.CatzMathUtils;
import frc.robot.Utils.Conversions;
import frc.robot.subsystems.drivetrain.ModuleIOInputsAutoLogged;


public class SwerveModule
{
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged   inputs = new ModuleIOInputsAutoLogged();

    private final int MOTOR_ID;


    private PIDController pid;
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

    private final double kP = 0.01;
    private final double kI = 0.0;
    private final double kD = 0.0;

    private double angleError = 0.0;
    private double flippedAngleError = 0.0;

    private double command;
    public boolean driveDirectionFlipped = false;

    private double wheelOffset;


    private int index;

    private double gyroAngle;


    public SwerveModule(int driveMotorID, int steerMotorID, int encoderDIOChannel, double offset, int index)
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

    public void setDrivePower(double pwr)
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

    public double getDrvDistance()
    {
        if(driveDirectionFlipped)
        {
            return inputs.driveMtrSensorPosition;
        }
        else
        {
            return -inputs.driveMtrSensorPosition;
        }
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

    public double getDriveDistanceInch()
    {
        return inputs.driveMtrSensorPosition * CatzConstants.DriveConstants.SDS_L1_GEAR_RATIO * CatzConstants.DriveConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE / 2048.0;
    }


    public double getDrvVelocity()
    {
        return inputs.driveMtrVelocity;
    }
    
    public double getAbsEncRadians()
    {
        return inputs.magEncoderValue * 2 * Math.PI;
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

        if (Math.abs(state.speedMetersPerSecond) < 0.001) 
        {
            setDrivePower(0.0);
            setSteerPower(0.0);
            return;
        }

        //optimize wheel angles
        state = SwerveModuleState.optimize(state, getModuleState().angle);
        
        //calculate pwrs
        double drivePwr = state.speedMetersPerSecond/DriveConstants.MAX_SPEED;
        double steerPIDpwr = pid.calculate(getAbsEncRadians(), state.angle.getRadians());

        //set powers
        setDrivePower(drivePwr);// + driveFeedforward);
        setSteerPower(steerPIDpwr);// + turnFeedforward);

        //logging
        Logger.getInstance().recordOutput("current roation" + Integer.toString(index), getAbsEncRadians());
        Logger.getInstance().recordOutput("target Angle" + Integer.toString(index), state.angle.getRadians());
       // Logger.getInstance().recordOutput("rotation" + Integer.toString(index), null);
    }


    public void resetMagEnc()
    {
        //tbd
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
    private Rotation2d getCurrentRotation()
    {
        return new Rotation2d(getAbsEncRadians());
    }

    public SwerveModuleState getModuleState()
    {
        double velocity = Conversions.falconToMPS(inputs.driveMtrSensorPosition,CatzConstants.DriveConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE, CatzConstants.DriveConstants.SDS_L2_GEAR_RATIO);
        
        return new SwerveModuleState(velocity, getCurrentRotation());
    }

    public SwerveModulePosition getModulePosition()
    {
        return new SwerveModulePosition(getDriveDistanceMeters(), getCurrentRotation());
    }
    
    public double getDriveDistanceMeters()
    {
        return  ((inputs.driveMtrSensorPosition / 2048)/ CatzConstants.DriveConstants.SDS_L2_GEAR_RATIO) * CatzConstants.DriveConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE;
    }
}
