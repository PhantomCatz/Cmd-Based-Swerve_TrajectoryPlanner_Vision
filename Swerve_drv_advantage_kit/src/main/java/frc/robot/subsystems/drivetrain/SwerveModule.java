/*
 * Orginal swerve module class taken from Timed Atlas code
 * 
 * 
 */
package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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

        //edit outputs all output collection should be declared here.
        Logger.getInstance().recordOutput("motor value test", inputs.driveMtrPercentOutput);
    }

    public void setWheelAngle(double target, double gyroAngle)
    {
        currentAngle = ((magEnc.get() - offset) * 360.0) - gyroAngle;
        // find closest angle to target angle
        angleError = closestAngle(currentAngle, target);

        // find closest angle to target angle + 180
        flippedAngleError = closestAngle(currentAngle, target + 180.0);

        // if the closest angle to target is shorter
        if (Math.abs(angleError) <= Math.abs(flippedAngleError))
        {
            driveDirectionFlipped = false;
            command = pid.calculate(currentAngle, currentAngle + angleError);
        }
        // if the closest angle to target + 180 is shorter
        else
        {
            driveDirectionFlipped = true;
            command = pid.calculate(currentAngle, currentAngle + flippedAngleError);
        }

        command = -command / (180 * kP); //scale down command to a range of -1 to 1
        io.setSteerPwrIO(command);
    }

    public void setSteerPower(double pwr)
    {
        switch (CatzConstants.currentMode)
        {
        case REAL:  io.setSteerPwrIO(pwr); 
            break;
        case SIM:   io.setSteerSimPwrIO(pwr);
            break;
        default:    io.setSteerPwrIO(pwr);
            break;
        }
    }

    public void setDrivePower(double pwr)
    {
        if(driveDirectionFlipped == true)
        {
            pwr = -pwr;
        }
        switch (CatzConstants.currentMode)
        {
        case REAL:  io.setDrivePwrPercentIO(-pwr); 
            break;
        case SIM:   io.setDriveSimPwrIO(-pwr);
            break;
        default:    io.setDrivePwrPercentIO(-pwr);
            break;
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
        return inputs.driveMtrSensorPosition * CatzConstants.SDS_L1_GEAR_RATIO * CatzConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE / 2048.0;
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

    //utils
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
