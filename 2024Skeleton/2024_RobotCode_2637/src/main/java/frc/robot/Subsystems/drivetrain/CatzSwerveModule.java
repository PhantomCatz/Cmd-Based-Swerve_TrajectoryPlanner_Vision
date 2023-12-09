/*
 * Orginal swerve module class taken from Timed Atlas code
 * 
 * 
 */
package frc.robot.Subsystems.drivetrain;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenixpro.controls.VelocityTorqueCurrentFOC;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.DriveConstants;
import frc.robot.Utils.CatzMathUtils;
import frc.robot.Utils.Conversions;


public class CatzSwerveModule {

    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged   inputs = new ModuleIOInputsAutoLogged();

    private PIDController m_pid;
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

    private final double kP = 0.4; //cuz error is in tenths place so no need to mutiply kp value
    private final double kI = 0.0;
    private final double kD = 0.0;

    private double m_wheelOffset;

    private int m_index;

    private VelocityTorqueCurrentFOC velocitySetter = new VelocityTorqueCurrentFOC(0);


    public CatzSwerveModule(int driveMotorID, int steerMotorID, int encoderDIOChannel, double offset, int index) {
        this.m_index = index;

        switch (CatzConstants.currentMode) {
            case REAL: io = new ModuleIOReal(driveMotorID, steerMotorID, encoderDIOChannel);
                break;
            case SIM : io = new ModuleIOSim();
                break;
            case REAL_WITH_PRO : io = new ModuleIOPro(driveMotorID, steerMotorID, encoderDIOChannel);
                break;
            default : io = new ModuleIOReal(driveMotorID, steerMotorID, encoderDIOChannel) {};
                break;
        }

        m_pid = new PIDController(kP, kI, kD);

        m_wheelOffset = offset;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Drive/Module " + Integer.toString(m_index), inputs);

        //Logging outputs
        Logger.getInstance().recordOutput("absenctorad" + Integer.toString(m_index) , getAbsEncRadians());

        SmartDashboard.putNumber("absenctorad" + Integer.toString(m_index) , getAbsEncRadians());
    }

    //----------------------------------------Setting pwr methods
    private void setSteerPower(double pwr) {
        if(CatzConstants.currentMode == CatzConstants.Mode.SIM) {
           io.setSteerSimPwrIO(pwr);
        }
        else {       
            io.setSteerPwrIO(pwr);
        }
    }

    private void setSteerVoltage(double volts) {
        if(CatzConstants.currentMode == CatzConstants.Mode.SIM) {

        }
        else {       
            io.setSteerVoltageIO(volts);
        }
    }

    private void setDrivePercent(double pwr) {
        if(CatzConstants.currentMode == CatzConstants.Mode.SIM) {
           io.setDriveSimPwrIO(pwr);
        }
        else {       
            io.setDrivePwrPercentIO(pwr);
        }
    }

    private void setDriveVelocity(double velocity) {
        if(CatzConstants.currentMode == CatzConstants.Mode.SIM) {
            
        }
        else {
            io.setDriveVelocityIO(velocity);
        }
    }
    //----------------------------------Util Methods catzswerve------------------------
    public double getDrvDistanceRaw() {
        return inputs.driveMtrSensorPosition;
    }

    public void setCoastMode() {
        io.setSteerCoastModeIO();
    }

    public void setBrakeMode() {
        io.setSteerBrakeModeIO();
    }

    public double getDrvVelocity() {
        return inputs.driveMtrVelocity;
    }
    
    private double getAbsEncRadians() {
        return (inputs.magEncoderValue - m_wheelOffset) * 2 * Math.PI;
    }
    //TBD put in autobalance file
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

        state = CatzMathUtils.optimize(state, getCurrentRotation());
        
        //ff control
        double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
        //double turnFeedforward = m_turnFeedforward.calculate(m_pid.getSetpoint().velocity);

        //calculate turn voltage
        double steerPIDpwr = - m_pid.calculate(getAbsEncRadians(), state.angle.getRadians()); 
        setSteerPower(steerPIDpwr);

        //calculate drive pwr
        double drivePwrVelocity = Conversions.MPSToFalcon(state.speedMetersPerSecond, 
                                                          DriveConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE, 
                                                          DriveConstants.SDS_L2_GEAR_RATIO); //to set is as a gear reduction not an overdrive
        //set power for FOC(experimental)
        if(CatzConstants.currentMode == CatzConstants.Mode.REAL_WITH_PRO) { 
            io.setDriveControlIO(velocitySetter.withVelocity(drivePwrVelocity));   
        }
        else { //set pwr for REAL/SIM
            setDriveVelocity(drivePwrVelocity); //+ driveFeedforward);
        }

        //logging
        Logger.getInstance().recordOutput("Drive/current roation" + Integer.toString(m_index), getAbsEncRadians());
        Logger.getInstance().recordOutput("Drive/target Angle" + Integer.toString(m_index), state.angle.getRadians());
        Logger.getInstance().recordOutput("Drive/drive velocity" + Integer.toString(m_index), drivePwrVelocity);
        Logger.getInstance().recordOutput("Drive/turn power" + Integer.toString(m_index), steerPIDpwr);
       // Logger.getInstance().recordOutput("rotation" + Integer.toString(index), d);
    }

    public void resetDriveEncs() {
        io.setDrvSensorPositionIO(0.0);
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
        return new SwerveModulePosition(- getDriveDistanceMeters(), getCurrentRotation());
    }
    
    public double getDriveDistanceMeters() {
        return  ((inputs.driveMtrSensorPosition / 2048)/ DriveConstants.SDS_L2_GEAR_RATIO) * DriveConstants.DRVTRAIN_WHEEL_CIRCUMFERENCE;
    }
}
