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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CatzConstants;
import frc.robot.Utils.CatzMathUtils;
import frc.robot.subsystems.drivetrain.ModuleIOInputsAutoLogged;


public class SwerveModule
{
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged   inputs = new ModuleIOInputsAutoLogged();

    private final int MOTOR_ID;

    private DutyCycleEncoder magEnc;
    private DigitalInput MagEncPWMInput;

    private PIDController pid;
    private final double kP = 0.01;
    private final double kI = 0.0;
    private final double kD = 0.0;

    private double currentAngle = 0.0;
    private double angleError = 0.0;
    private double flippedAngleError = 0.0;

    private double command;
    public boolean driveDirectionFlipped = false;

    private double offset;

    private int index;


    public SwerveModule(int driveMotorID, int steerMotorID, int encoderDIOChannel, double offset, int index)
    {
        this.index = index;
        
        MagEncPWMInput = new DigitalInput(encoderDIOChannel);
        magEnc = new DutyCycleEncoder(MagEncPWMInput);

        switch (CatzConstants.currentMode)
        {
            case REAL:
                    io = new ModuleIOReal(driveMotorID, steerMotorID, magEnc);
                break;
            case SIM :
                    io = new ModuleIOSim();
                break;
            default :
                    io = new ModuleIOReal(driveMotorID, steerMotorID, magEnc) {};
                break;
        }

        pid = new PIDController(kP, kI, kD);

        this.offset = offset;

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
        if(driveDirectionFlipped == true)
        {
            pwr = -pwr;
        }
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
    public void resetMagEnc()
    {
        io.resetMagEncoderIO();
    }
    
    public void resetDriveEncs()
    {
        io.resetDrvSensorPositionIO();
    }

    public void initializeOffset()
    {
       offset = inputs.magEncoderValue;
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

        io.resetDrvSensorPositionIO();
        while(Math.abs(inputs.driveMtrSensorPosition) > 1.0)
        {
            i++;
            if(i >= 3000)
            {
                resetDrvDistance();
            }
        }
    }

    private Rotation2d getCurrentRotation()
    {
        return Rotation2d.fromDegrees(inputs.magEncoderValue - offset);
    }

    public SwerveModulePosition getModulePosition()
    {
        return new SwerveModulePosition(Units.inchesToMeters(getDriveDistanceInch()), Rotation2d.fromDegrees(getCurrentRotation().getDegrees()));
    }

    public double getDriveDistanceInch()
    {
        return inputs.driveMtrSensorPosition * CatzConstants.DriveConstants.SDS_L1_GEAR_RATIO * CatzConstants.DriveConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE / 2048.0;
    }


    public double getDrvVelocity()
    {
        return inputs.driveMtrVelocity;
    }
    
    public double getMagEncAngle()
    {
        return inputs.magEncoderValue;
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

    public void setDesiredState(SwerveModuleState desiredState)
    {
        desiredState = SwerveModuleState.optimize(desiredState, getCurrentRotation());
        setDrivePower(desiredState.speedMetersPerSecond / CatzConstants.DriveConstants.MAX_SPEED);

        double targetAngle = (Math.abs(desiredState.speedMetersPerSecond) 
                                <= (CatzConstants.DriveConstants.MAX_SPEED * 0.01)) 
                                    ? getCurrentRotation().getDegrees() : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.

        setSteerPower(pid.calculate(getCurrentRotation().getDegrees(), targetAngle));
    }
}
