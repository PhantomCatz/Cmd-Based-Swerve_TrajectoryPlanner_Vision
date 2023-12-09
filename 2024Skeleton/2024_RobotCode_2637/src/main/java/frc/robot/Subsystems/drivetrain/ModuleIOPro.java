package frc.robot.Subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenixpro.configs.CurrentLimitsConfigs;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenixpro.BaseStatusSignalValue;
import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.subsystems.drivetrain.ModuleIO.ModuleIOInputs;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class ModuleIOPro implements ModuleIO{
    private final CANSparkMax STEER_MOTOR;
    private final TalonFX DRIVE_MOTOR;

    private DutyCycleEncoder magEnc;
    private DigitalInput MagEncPWMInput;

    //encoder setups
    private StatusSignalValue<Double> drivePosition;
    private StatusSignalValue<Double> driveVelocity;
    private BaseStatusSignalValue[] signals;

    //current limiting
    private SupplyCurrentLimitConfiguration swerveModuleCurrentLimit;
    private final int     CURRENT_LIMIT_AMPS            = 55;
    private final int     CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;

    private final int     STEER_CURRENT_LIMIT_AMPS      = 30;
    private final double  NEUTRAL_TO_FULL_SECONDS       = 0.1;

    private PositionVoltage angleSetter = new PositionVoltage(0, true, 0, 0, true);
    private VelocityTorqueCurrentFOC velocitySetter = new VelocityTorqueCurrentFOC(0);
    private StatusCode initializationStatus = StatusCode.StatusCodeNotInitialized;

    public ModuleIOPro(int driveMotorIDIO, int steerMotorIDIO, int magDIOPort) {
        //--------------------Mag Encoder
        MagEncPWMInput = new DigitalInput(magDIOPort);
        magEnc = new DutyCycleEncoder(MagEncPWMInput);

        //---------------------Drive Mtr----------------------------
        DRIVE_MOTOR = new TalonFX(driveMotorIDIO);
        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
        Slot0Configs driveConfigs = new Slot0Configs();
        driveConfigs.kP = 0.1;
        talonConfigs.Slot0 = driveConfigs;
        talonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //current limiting
        talonConfigs.CurrentLimits = new CurrentLimitsConfigs();
        talonConfigs.CurrentLimits.SupplyCurrentLimitEnable = ENABLE_CURRENT_LIMIT;
        talonConfigs.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT_AMPS;


        //-----------------Steer Motor---------------------------------------
        STEER_MOTOR = new CANSparkMax(steerMotorIDIO, MotorType.kBrushless);
        STEER_MOTOR.restoreFactoryDefaults();

        STEER_MOTOR.setSmartCurrentLimit(STEER_CURRENT_LIMIT_AMPS);

        STEER_MOTOR.setIdleMode(IdleMode.kCoast);


        //apply drive motor configs
        for(int i=0;i<5;i++){
            initializationStatus = DRIVE_MOTOR.getConfigurator().apply(talonConfigs);
            if(initializationStatus.isOK())
                break;
            else if(!initializationStatus.isOK())
                System.out.println("Failed to Configure CAN ID" + driveMotorIDIO);
            
        }

        drivePosition = DRIVE_MOTOR.getPosition();
        driveVelocity = DRIVE_MOTOR.getVelocity();

        signals = new BaseStatusSignalValue[2];
        signals[0] = drivePosition;
        signals[1] = driveVelocity;
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveMtrVelocity = driveVelocity.getValue();
        inputs.driveMtrSensorPosition = drivePosition.getValue();
        inputs.magEncoderValue = magEnc.get();
    }

    @Override
    public void setDriveControlIO(VelocityTorqueCurrentFOC controlValue) {
        DRIVE_MOTOR.setControl(controlValue);
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
        DRIVE_MOTOR.setRotorPosition(sensorPos);
       // DRIVE_MOTOR.setSelectedSensorPosition(0.0);
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
