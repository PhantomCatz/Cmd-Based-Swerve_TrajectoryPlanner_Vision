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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CatzConstants;
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

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState, double gyroAngle) {
        this.gyroAngle = gyroAngle;
        var encoderRotation = inputs.magEncoderValue; 

        // Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState state = desiredState; //SwerveModuleState.optimize(desiredState, encoderRotation);

        final double driveOutput =
            state.speedMetersPerSecond;


        final double turnFeedforward =
            m_turnFeedforward.calculate(pid.getSetpoint());

            double currentAngle = ((inputs.magEncoderValue - wheelOffset) * 360.0) - gyroAngle;
            // find closest angle to target angle
            angleError = closestAngle(currentAngle, state.angle.getDegrees());
    
            // find closest angle to target angle + 180
            flippedAngleError = closestAngle(currentAngle, state.angle.getDegrees() + 180.0);
    
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
            
        setDrivePower(driveOutput);// + driveFeedforward);
        setSteerPower(command);// + turnFeedforward);

        Logger.getInstance().recordOutput("current roation" + Integer.toString(index), encoderRotation);
        Logger.getInstance().recordOutput("target Angle" + Integer.toString(index), desiredState.angle.getDegrees());
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
        //
    }

    //inputs the rotation object as degree
    private Rotation2d getCurrentRotation()
    {
        return Rotation2d.fromDegrees(((inputs.magEncoderValue*360)));
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
