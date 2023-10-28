package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class ModuleIOReal implements ModuleIO
{

    private final CANSparkMax STEER_MOTOR;
    private final WPI_TalonFX DRIVE_MOTOR;

    private DutyCycleEncoder magEnc;
    private DigitalInput MagEncPWMInput;

    public boolean driveDirectionFlipped = false;

    //current limiting
    private SupplyCurrentLimitConfiguration swerveModuleCurrentLimit;
    private final int     CURRENT_LIMIT_AMPS            = 55;
    private final int     CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;

    private final int     STEER_CURRENT_LIMIT_AMPS      = 30;

    public ModuleIOReal(int driveMotorIDIO, int steerMotorIDIO, int magDIOPort)
    {
        MagEncPWMInput = new DigitalInput(magDIOPort);
        magEnc = new DutyCycleEncoder(MagEncPWMInput);

        STEER_MOTOR = new CANSparkMax(steerMotorIDIO, MotorType.kBrushless);
        DRIVE_MOTOR = new WPI_TalonFX(driveMotorIDIO);

        STEER_MOTOR.restoreFactoryDefaults();
        DRIVE_MOTOR.configFactoryDefault();

        //Set current limit
        swerveModuleCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);

        STEER_MOTOR.setSmartCurrentLimit(STEER_CURRENT_LIMIT_AMPS);
        DRIVE_MOTOR.configSupplyCurrentLimit(swerveModuleCurrentLimit);

        STEER_MOTOR.setIdleMode(IdleMode.kCoast);
        DRIVE_MOTOR.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs)
    {
        inputs.driveMtrVelocity = DRIVE_MOTOR.getSelectedSensorVelocity();
        inputs.driveMtrSensorPosition = DRIVE_MOTOR.getSelectedSensorPosition();
        inputs.magEncoderValue = magEnc.get();
        inputs.driveMtrPercentOutput = DRIVE_MOTOR.getMotorOutputPercent();
        inputs.turnMtrEnc = STEER_MOTOR.get(); //not correct TBD
    }

    @Override
    public void setDrivePwrPercentIO(double drivePwrPercent) 
    {
        DRIVE_MOTOR.set(ControlMode.PercentOutput, drivePwrPercent);
    }

    @Override
    public void setSteerPwrIO(double SteerPwr) 
    {
        STEER_MOTOR.set(SteerPwr);
    }

    @Override
    public void setSteerCoastModeIO() 
    {
        STEER_MOTOR.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public void setSteerBrakeModeIO() 
    {
        STEER_MOTOR.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void setDrvSensorPositionIO(double sensorpos)
    {
        DRIVE_MOTOR.setSelectedSensorPosition(0.0);
    }
    @Override
    public void reverseDriveIO(boolean enable)
    {
        DRIVE_MOTOR.setInverted(enable);
    }
    
    @Override
    public void resetMagEncoderIO()
    {
        magEnc.reset();
    }

}
