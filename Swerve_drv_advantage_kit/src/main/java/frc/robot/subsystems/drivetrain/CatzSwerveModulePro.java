/*
 * Orginal swerve module class taken from Timed Atlas code
 * 
 * 
 */
package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.controls.VelocityTorqueCurrentFOC;

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
import frc.robot.Utils.CTREUtils;
import frc.robot.Utils.CatzMathUtils;
import frc.robot.Utils.Conversions;
import frc.robot.subsystems.drivetrain.ModuleIOInputsAutoLogged;


public class CatzSwerveModulePro {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged   inputs = new ModuleIOInputsAutoLogged();

    private final int MOTOR_ID;

    private PIDController m_pid;
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

    private final double kP = 0.4; //cuz error is in tenths place so no need to mutiply kp value
    private final double kI = 0.0;
    private final double kD = 0.0;

    private double m_wheelOffset;

    private int m_index;

    private PositionVoltage angleSetter = new PositionVoltage(0, true, 0, 0, true);
    private VelocityTorqueCurrentFOC velocitySetter = new VelocityTorqueCurrentFOC(0);


    public CatzSwerveModulePro(int driveMotorID, int steerMotorID, int encoderDIOChannel, double offset, int index) {
        this.m_index = index;

        switch (CatzConstants.currentMode) {
            case REAL: io = new ModuleIOReal(driveMotorID, steerMotorID, encoderDIOChannel);
                break;
            case SIM : io = new ModuleIOSim();
                break;
            default : io = new ModuleIOReal(driveMotorID, steerMotorID, encoderDIOChannel) {};
                break;
        }

        m_pid = new PIDController(kP, kI, kD);

        m_wheelOffset = offset;

        //for shuffleboard
        MOTOR_ID = steerMotorID;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Drive/Module " + Integer.toString(m_index), inputs);

        //Logging outputs
        Logger.getInstance().recordOutput("absenctorad" + Integer.toString(m_index) , getAbsEncRadians());

        SmartDashboard.putNumber("absenctorad" + Integer.toString(m_index) , getAbsEncRadians());
    }



    public void setSteerPower(double pwr) {
        if(CatzConstants.currentMode == CatzConstants.Mode.SIM) {
           io.setSteerSimPwrIO(pwr);
        }
        else {       
            io.setSteerPwrIO(pwr);
        }
    }

    public void setSteerVoltage(double volts) {
        if(CatzConstants.currentMode == CatzConstants.Mode.SIM) {

        }
        else {       
            io.setSteerVoltageIO(volts);
        }
    }

    public void setDrivePercent(double pwr) {
        if(CatzConstants.currentMode == CatzConstants.Mode.SIM) {
           io.setDriveSimPwrIO(pwr);
        }
        else {       
            io.setDrivePwrPercentIO(pwr);
        }
    }

    public void setDriveVelocity(double velocity) {
        if(CatzConstants.currentMode == CatzConstants.Mode.SIM) {
            
        }
        else {
            io.setDriveVelocityIO(velocity);
        }
    }

    public double getDrvDistanceRaw() {
        return inputs.driveMtrSensorPosition;
    }

    public void setCoastMode() {
        io.setSteerCoastModeIO();
    }

    public void setBrakeMode() {
        io.setSteerBrakeModeIO();
    }

    public void resetDrvDistance() {
        int i = 0;

        io.setDrvSensorPositionIO(0.0); //resetsensorpos
        while(Math.abs(inputs.driveMtrSensorPosition) > 1.0) {
            i++;
            if(i >= 3000) {
                resetDrvDistance();
            }
        }
    }

    public double getDrvVelocity() {
        return inputs.driveMtrVelocity;
    }
    
    private double getAbsEncRadians() {
        return (inputs.magEncoderValue - m_wheelOffset) * 2 * Math.PI;
    }

    /*Auto Balance */
    public void reverseDrive(Boolean reverse) {
        io.reverseDriveIO(reverse);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param state Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState state) {
        //appy wheel optimizations
        state = CTREUtils.optimize(state, getCurrentRotation());

        //calculate turn voltage
        double steerPIDpwr = - m_pid.calculate(getAbsEncRadians(), state.angle.getRadians()); 
        double steerPIDVoltage = steerPIDpwr * 12; //convert to voltage
        
        double velocityToSet = state.speedMetersPerSecond; //tbd determine later// * //driveRotationsPerMeter;

        //set powers
        setSteerVoltage(steerPIDVoltage);
        io.setDriveControlIO(velocitySetter.withVelocity(velocityToSet));

        //logging
        Logger.getInstance().recordOutput("Drive/current roation" + Integer.toString(m_index), getAbsEncRadians());
        Logger.getInstance().recordOutput("Drive/target Angle" + Integer.toString(m_index), state.angle.getRadians());
        Logger.getInstance().recordOutput("Drive/turn power" + Integer.toString(m_index), steerPIDpwr);
       // Logger.getInstance().recordOutput("rotation" + Integer.toString(index), d);
    }


    public void resetMagEnc() {
        
    }

    public void resetDriveEncs() {
        io.setDrvSensorPositionIO(0.0);
    }

    public void initializeOffset() {
        //TBD
    }

    //inputs the rotation object as radian conversion
    public Rotation2d getCurrentRotation() {
        return new Rotation2d(getAbsEncRadians());
    }

    public SwerveModuleState getModuleState() {
        double velocity = Conversions.falconToMPS(inputs.driveMtrVelocity, DriveConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE, DriveConstants.SDS_L2_GEAR_RATIO);
        
        return new SwerveModuleState(velocity, getCurrentRotation());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDriveDistanceMeters(), getCurrentRotation());
    }
    
    public double getDriveDistanceMeters() {
        return  ((inputs.driveMtrSensorPosition / 2048)/ DriveConstants.SDS_L2_GEAR_RATIO) * DriveConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE;
    }
}