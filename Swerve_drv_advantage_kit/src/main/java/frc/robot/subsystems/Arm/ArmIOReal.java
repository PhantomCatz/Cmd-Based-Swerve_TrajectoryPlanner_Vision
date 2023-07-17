package frc.robot.subsystems.Arm;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;

public class ArmIOReal implements ArmIO
{
    private WPI_TalonFX armMtr;

    private final int ARM_MC_ID = 20;
  
    private final double EXTEND_PWR  = 0.2;
    private final double RETRACT_PWR = -0.2;
  
    //Conversion factors
  
    //current limiting
    private SupplyCurrentLimitConfiguration armCurrentLimit;
    private final int     CURRENT_LIMIT_AMPS            = 55;
    private final int     CURRENT_LIMIT_TRIGGER_AMPS    = 55;
    private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
    private final boolean ENABLE_CURRENT_LIMIT          = true;
  
    //gear ratio
    private final double VERSA_RATIO  = 7.0/1.0;
  
    private final double PUILEY_1      = 24.0;
    private final double PUILEY_2      = 18.0;
    private final double PUILEY_RATIO  = PUILEY_1 / PUILEY_2;
      
    private final double FINAL_RATIO   = VERSA_RATIO * PUILEY_RATIO;
    private final double FINAL_CIRCUMFERENCE = 3.54; 
  
  
    private final boolean LIMIT_SWITCH_IGNORED = false;
    private final boolean LIMIT_SWITCH_MONITORED = true;
  
    private final double CNTS_OVER_REV = 2048.0 / 1.0;
  
    private final double CNTS_PER_INCH_CONVERSION_FACTOR = CNTS_OVER_REV/FINAL_CIRCUMFERENCE;
  
    private final double POS_ENC_INCH_RETRACT = 0.0;
    private final double POS_ENC_INCH_EXTEND = 8.157;
    private final double POS_ENC_INCH_PICKUP = 4.157;
  
    private final double POS_ENC_CNTS_RETRACT  = 0.0+154;//POS_ENC_INCH_RETRACT * CNTS_PER_INCH_CONVERSION_FACTOR;
    private final double POS_ENC_CNTS_EXTEND  = 44000.0+154;//POS_ENC_INCH_EXTEND * CNTS_PER_INCH_CONVERSION_FACTOR;
    private final double POS_ENC_CNTS_PICKUP = 22000.0+154;//POS_ENC_INCH_PICKUP * CNTS_PER_INCH_CONVERSION_FACTOR;
  
  
    private boolean extendSwitchState = false;
  
    private int SWITCH_CLOSED = 1;
  
    private final double ARM_KP = 0.15;
    private final double ARM_KI = 0.0001;
    private final double ARM_KD = 0.0;
  
    private final double ARM_CLOSELOOP_ERROR = 3000;
  
    private final double MANUAL_CONTROL_PWR_OFF = 0.0;
  
    private boolean highExtendProcess = false;
  
    private double targetPosition = -999.0;
    private double currentPosition = -999.0;
    private double positionError = -999.0; 
    private double elevatorPosition = -999.0;
  
    private boolean armInPosition = false;
    private int numConsectSamples = 0;
  
    private final double ARM_POS_ERROR_THRESHOLD = 2700.0; //0.5 inches    previously 500 enc counts
    private final double NO_TARGET_POSITION = -999999.0;
  
    public ArmIOReal()
    {
        armMtr = new WPI_TalonFX(ARM_MC_ID);

        armMtr.configFactoryDefault();
    
        armMtr.setNeutralMode(NeutralMode.Brake);
        armMtr.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
        armMtr.overrideLimitSwitchesEnable(LIMIT_SWITCH_MONITORED);
    
        armMtr.config_kP(0, ARM_KP);
        armMtr.config_kI(0, ARM_KI);
        armMtr.config_kD(0, ARM_KD);
    
        armCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);
    
        armMtr.configSupplyCurrentLimit(armCurrentLimit);
    
        armMtr.configAllowableClosedloopError(0, ARM_CLOSELOOP_ERROR);
    
        armMtr.set(ControlMode.PercentOutput, MANUAL_CONTROL_PWR_OFF);
    
    }

    @Override
    public void updateInputs(ArmIOInputs inputs)
    {
        inputs.armMotorEncoder        = armMtr.getSelectedSensorPosition();
        inputs.isRevLimitSwitchClosed = (armMtr.getSensorCollection().isRevLimitSwitchClosed() == SWITCH_CLOSED);
        inputs.currentArmControlMode  = armMtr.getControlMode();
    }
    @Override
    public void setSelectedSensorPositionIO(double encoderResetPos) 
    {
        armMtr.setSelectedSensorPosition(encoderResetPos);
    }
    
    @Override
    public void setArmPwrIO(double pwr)
    {
        armMtr.set(ControlMode.PercentOutput, pwr);
    }

    @Override
    public void armSetFullExtendPosIO()
    {
        armMtr.set(ControlMode.Position, POS_ENC_CNTS_EXTEND);
    }

    @Override
    public void armSetRetractPosIO()
    {
        armMtr.set(ControlMode.Position, POS_ENC_CNTS_RETRACT);
    }

    @Override
    public void armSetPickupPosIO()
    {
        armMtr.set(ControlMode.Position, POS_ENC_CNTS_PICKUP);
    }


}
