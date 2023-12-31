package frc.robot.subsystems.Intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import frc.robot.Utils.CatzSharedDataUtil;

public class IntakeIOReal implements IntakeIO 
{

    //----------------------------------------------------------------------------------------------
    //
    //  Roller
    //
    //----------------------------------------------------------------------------------------------
    private WPI_TalonFX rollersMtr;

    private final int ROLLERS_MC_ID  = 30;
    
    //current limiting
    private SupplyCurrentLimitConfiguration rollerCurrentLimit;

    private final int     CURRENT_LIMIT_AMPS_ROLLER            = 40;    
    private final int     CURRENT_LIMIT_TRIGGER_AMPS_ROLLER    = 40;

    //----------------------------------------------------------------------------------------------
    //
    //  Wrist
    //
    //----------------------------------------------------------------------------------------------
    private WPI_TalonFX wristMtr;

    private final int    WRIST_MC_ID   = 31;

    //current limiting
    private SupplyCurrentLimitConfiguration wristCurrentLimit;

    private final int     CURRENT_LIMIT_AMPS            = 55;
    private final int     CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;

    private final double SOFT_LIMIT_FORWARD = 0.0;
    private final double SOFT_LIMIT_REVERSE = -8900.0;



    public IntakeIOReal()
    {
        //----------------------------------------------------------------------------------------------
        //  Roller
        //----------------------------------------------------------------------------------------------
        rollersMtr = new WPI_TalonFX(ROLLERS_MC_ID);
        rollersMtr.configFactoryDefault();
        rollersMtr.setNeutralMode(NeutralMode.Brake);

        rollerCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS_ROLLER, 
                                                                                       CURRENT_LIMIT_TRIGGER_AMPS_ROLLER, 
                                                                                       CURRENT_LIMIT_TIMEOUT_SECONDS);

        rollersMtr.configSupplyCurrentLimit(rollerCurrentLimit);

        //----------------------------------------------------------------------------------------------
        //  Wrist
        //----------------------------------------------------------------------------------------------
        wristMtr = new WPI_TalonFX(WRIST_MC_ID);

        wristMtr.configFactoryDefault();

        wristMtr.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);

        wristMtr.configIntegratedSensorOffset(0.0);
        
        wristMtr.setNeutralMode(NeutralMode.Brake);

        wristMtr.configForwardSoftLimitThreshold(SOFT_LIMIT_FORWARD);
        wristMtr.configReverseSoftLimitThreshold(SOFT_LIMIT_REVERSE);

        wristMtr.configForwardSoftLimitEnable(true);                  
        wristMtr.configReverseSoftLimitEnable(true);

        wristCurrentLimit  = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, 
                                                                                       CURRENT_LIMIT_TRIGGER_AMPS, 
                                                                                       CURRENT_LIMIT_TIMEOUT_SECONDS);        
        wristMtr.configSupplyCurrentLimit(wristCurrentLimit);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) 
    {
        inputs.wristPosEnc = wristMtr.getSelectedSensorPosition();
        CatzSharedDataUtil.sharedWristEncCnts = inputs.wristPosEnc;
        inputs.wristTemp = wristMtr.getTemperature();
    }

    @Override
    public void rollersOffIO() 
    {
        rollersMtr.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public void rollersOnIO(double rollerPwrIO) 
    {
        rollersMtr.set(ControlMode.PercentOutput, rollerPwrIO);
    }

    @Override
    public void rollerVoltageIO(double volts)
    {
        rollersMtr.setVoltage(volts);
    }

    @Override
    public void intakeManualHoldingIO(double targetHoldingPwrIO) 
    {
        wristMtr.set(ControlMode.PercentOutput, targetHoldingPwrIO);
    }

    @Override
    public void wristSetPercentOuputIO(double setIntakeMtrPwrIO) 
    {
        wristMtr.set(ControlMode.PercentOutput, setIntakeMtrPwrIO);
    }

    @Override
    public void intakeConfigureSoftLimitOverride(boolean enabled)
    {
        wristMtr.configForwardSoftLimitEnable(enabled);
        wristMtr.configReverseSoftLimitEnable(enabled);
    }
}
