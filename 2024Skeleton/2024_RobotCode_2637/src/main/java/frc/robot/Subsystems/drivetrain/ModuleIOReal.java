package frc.robot.Subsystems.drivetrain;

import com.ctre.phoenix6.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix6.*;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.CatzConstants.DriveConstants;
import com.ctre.phoenix6.BaseStatusSignal;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class ModuleIOReal implements ModuleIO {

    private final CANSparkMax STEER_MOTOR;
    private final WPI_TalonFX DRIVE_MOTOR;

    private DutyCycleEncoder magEnc;
    private DigitalInput MagEncPWMInput;

    //current limiting
    private SupplyCurrentLimitConfiguration swerveModuleCurrentLimit;


    public ModuleIOReal(int driveMotorIDIO, int steerMotorIDIO, int magDIOPort) {
        MagEncPWMInput = new DigitalInput(magDIOPort);
        magEnc = new DutyCycleEncoder(MagEncPWMInput);

        STEER_MOTOR = new CANSparkMax(steerMotorIDIO, MotorType.kBrushless);
        DRIVE_MOTOR = new WPI_TalonFX(driveMotorIDIO);

        STEER_MOTOR.restoreFactoryDefaults();
        DRIVE_MOTOR.configFactoryDefault();

        //Set current limit
        swerveModuleCurrentLimit = new SupplyCurrentLimitConfiguration(DriveConstants.ENABLE_CURRENT_LIMIT, 
                                                                       DriveConstants.CURRENT_LIMIT_AMPS, 
                                                                       DriveConstants.CURRENT_LIMIT_TRIGGER_AMPS, 
                                                                       DriveConstants.CURRENT_LIMIT_TIMEOUT_SECONDS);

        STEER_MOTOR.setSmartCurrentLimit(DriveConstants.STEER_CURRENT_LIMIT_AMPS);
        DRIVE_MOTOR.configSupplyCurrentLimit(swerveModuleCurrentLimit);

        STEER_MOTOR.setIdleMode(IdleMode.kCoast);
        DRIVE_MOTOR.setNeutralMode(NeutralMode.Brake);

        DRIVE_MOTOR.config_kP(0, 0.1);
        DRIVE_MOTOR.config_kI(0, 0.0);
        DRIVE_MOTOR.config_kD(0, 0.0);
        DRIVE_MOTOR.configClosedloopRamp(DriveConstants.NEUTRAL_TO_FULL_SECONDS);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveMtrVelocity = DRIVE_MOTOR.getSelectedSensorVelocity();
        inputs.driveMtrSensorPosition = DRIVE_MOTOR.getSelectedSensorPosition();
        inputs.magEncoderValue = magEnc.get();
        inputs.driveMtrPercentOutput = DRIVE_MOTOR.getMotorOutputPercent();
    }

    @Override
    public void setDriveVelocityIO(double velocity) {
        //negative to align with Controler TBD?
        DRIVE_MOTOR.set(ControlMode.Velocity, - velocity * DriveConstants.VEL_FF);
    }

    @Override
    public void setDrivePwrPercentIO(double drivePwrPercent) {
        //negative to align with Controler TBD?
        DRIVE_MOTOR.set(ControlMode.PercentOutput, - drivePwrPercent);
    }

    @Override
    public void setSteerPwrIO(double SteerPwr) {
        STEER_MOTOR.set(SteerPwr);
    }

    @Override
    public void setSteerVoltageIO(double steerVoltage) {
        STEER_MOTOR.setVoltage(steerVoltage);
    }

    @Override
    public void setSteerCoastModeIO() {
        STEER_MOTOR.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public void setSteerBrakeModeIO() {
        STEER_MOTOR.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void setDrvSensorPositionIO(double sensorPos) {
        DRIVE_MOTOR.setSelectedSensorPosition(0.0);
    }
    @Override
    public void reverseDriveIO(boolean enable) {
        DRIVE_MOTOR.setInverted(enable);
    }
    
    @Override
    public void resetMagEncoderIO() {
        magEnc.reset();
    }

}
