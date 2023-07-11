// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.Robot;
//import frc.robot.Robot.mechMode;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;




public class CatzElevatorSubsystem extends SubsystemBase {

  private final ElevatorIO io;
  private final EleavtorIOInputsAutoLogged  inputs = new EleavtorIOInputsAutoLogged();

  private static CatzElevatorSubsystem instance;

  private WPI_TalonFX elevatorMtr;

  private final int ELEVATOR_MC_ID = 10;

  private final double MAX_MANUAL_SCALED_POWER = 0.7;

  private final double MANUAL_CONTROL_DEADBAND = 0.1;

  private final double MANUAL_CONTROL_PWR_OFF = 0.0;

  private boolean elevatorInManual = false;
  private double targetPositionEnc;

  private final double MANUAL_HOLD_STEP_SIZE = 10000.0; //5000.0;
  

  //constants foir calc encoder to inch

  private final double FIRST_GEAR1 = 13;
  private final double FIRST_GEAR2 = 48;
  private final double FIRST_GEAR_RATIO = FIRST_GEAR2/FIRST_GEAR1;

  private final double HTD1 = 15;
  private final double HTD2 = 30;
  private final double HTD_RATIO = HTD2/HTD1;

  private final double SECOND_GEAR1 = 28;
  private final double SECOND_GEAR2 = 24;
  private final double SECOND_GEAR_RATIO = SECOND_GEAR2/SECOND_GEAR1;

  private final double FINAL_RATIO = FIRST_GEAR_RATIO*HTD_RATIO*SECOND_GEAR_RATIO;

  private final double CNTS_TO_REV = 2048/1;

  private final double SPROKET_DIAMETER = 1.751;
  private final double SPROKET_CIRCUMFERENCE = SPROKET_DIAMETER*Math.PI;

  private final double INCHES_TO_COUNTS_CONVERSTION_FACTOR = ((CNTS_TO_REV*FINAL_RATIO)/SPROKET_CIRCUMFERENCE);
  

  


  //current limiting
  private SupplyCurrentLimitConfiguration elevatorCurrentLimit;
  private final int     CURRENT_LIMIT_AMPS            = 55;
  private final int     CURRENT_LIMIT_TRIGGER_AMPS    = 55;
  private final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
  private final boolean ENABLE_CURRENT_LIMIT          = true;



  private final int SWITCH_CLOSED = 1;

  private boolean lowSwitchState  = false;
  private boolean highSwitchState = false;



  private final boolean LIMIT_SWITCH_IGNORED   = false;
  private final boolean LIMIT_SWITCH_MONITORED = true;  // limit switches will shut off the motor


  private final double POS_ENC_INCH_LOW  = 0.0;
  private final double POS_ENC_INCH_MID_CONE  = 39.616;
  private final double POS_ENC_INCH_MID_CUBE  = 26;
  private final double POS_ENC_INCH_HIGH = 47.187;

  private final double POS_ENC_CNTS_LOW  = POS_ENC_INCH_LOW * INCHES_TO_COUNTS_CONVERSTION_FACTOR;
  private final double POS_ENC_CNTS_MID_CONE  = POS_ENC_INCH_MID_CONE * INCHES_TO_COUNTS_CONVERSTION_FACTOR;//91000.0;// needs to be lower...too high
  // private final double POS_ENC_CNTS_MID_CUBE  = POS_ENC_INCH_MID_CUBE * INCHES_TO_COUNTS_CONVERSTION_FACTOR;
  private final double POS_ENC_CNTS_MID_CUBE = 50000.0;

  private final double POS_ENC_CNTS_HIGH = POS_ENC_INCH_HIGH * INCHES_TO_COUNTS_CONVERSTION_FACTOR;//111200.0;
  
  private final double ELEVATOR_KP_LOW = 0.03;
  private final double ELEVATOR_KI_LOW = 0.0002;
  private final double ELEVATOR_KD_LOW = 0.001;

  private final double ELEVATOR_KP_MID = 0.083;
  private final double ELEVATOR_KI_MID = 0.0002;
  private final double ELEVATOR_KD_MID = 0.0;

  private final double ELEVATOR_KP_HIGH = ELEVATOR_KP_MID;
  private final double ELEVATOR_KI_HIGH = ELEVATOR_KI_MID;
  private final double ELEVATOR_KD_HIGH = ELEVATOR_KD_MID;

  private final double ARM_ENCODER_THRESHOLD = 35000.0;

  private final double HOLDING_FEED_FORWARD = 0.044;


  private final double CLOSELOOP_ERROR_THRESHOLD_LOW = 50; 
 // private final double CLOSELOOP_ERROR_THRESHOLD_HIGH_MID = 300; 
  private final double CLOSELOOP_ERROR_THRESHOLD_HIGH_MID = 225; 


  private Timer elevatorTime;

  public boolean armRetractingAndElevatorDescent = false;

  private double targetPosition = -999.0;
  private double currentPosition = -999.0;
  private double positionError = -999.0;

  private final double ELEVATOR_POS_ERROR_THRESHOLD = 1000.0; //0.424 inches

  private final double NO_TARGET_POSITION = -999999.0;

  private boolean elevatorInPosition = false;

  private int numConsectSamples = 0;



  /** Creates a new CatzElevatorSubsystem. */
  public CatzElevatorSubsystem() 
  {
    switch(CatzConstants.currentMode)
    {
        case REAL:
            io = new ElevatorIOReal();
            break;
        case SIM :
            io = new ElevatorIOSim();
            break;
        default:
            io = new ElevatorIOReal() {};
            break;
    }


    elevatorMtr = new WPI_TalonFX(ELEVATOR_MC_ID);

    elevatorMtr.configFactoryDefault();

    elevatorMtr.setNeutralMode(NeutralMode.Brake);

    elevatorMtr.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    elevatorMtr.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    elevatorMtr.overrideLimitSwitchesEnable(LIMIT_SWITCH_MONITORED);

    elevatorCurrentLimit = new SupplyCurrentLimitConfiguration(ENABLE_CURRENT_LIMIT, CURRENT_LIMIT_AMPS, CURRENT_LIMIT_TRIGGER_AMPS, CURRENT_LIMIT_TIMEOUT_SECONDS);

    elevatorMtr.configSupplyCurrentLimit(elevatorCurrentLimit);


    elevatorMtr.config_kP(0, ELEVATOR_KP_HIGH);
    elevatorMtr.config_kI(0, ELEVATOR_KI_HIGH);
    elevatorMtr.config_kD(0, ELEVATOR_KD_HIGH);



    elevatorMtr.config_IntegralZone(0, 2000.0);//TBD should go away once feet foward

    elevatorMtr.selectProfileSlot(0, 0);

    elevatorMtr.configAllowableClosedloopError(0, CLOSELOOP_ERROR_THRESHOLD_HIGH_MID);//make this constant and make values in inches
    
    elevatorMtr.set(ControlMode.PercentOutput, MANUAL_CONTROL_PWR_OFF);

    elevatorTime = new Timer();
    elevatorTime.reset();
    elevatorTime.start();
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
    currentPosition = elevatorMtr.getSelectedSensorPosition();
    positionError = currentPosition - targetPosition;

    if((Math.abs(positionError) <= ELEVATOR_POS_ERROR_THRESHOLD) && targetPosition != NO_TARGET_POSITION)
    {
        targetPosition = NO_TARGET_POSITION;
        numConsectSamples++;
            if(numConsectSamples >= 10)
            {   
                elevatorInPosition = true;
            }
        }
    else
    {
        numConsectSamples = 0;
    }
  }
    public void elevatorManual(double pwr)
    {
        double mtrPower;

        mtrPower = pwr * MAX_MANUAL_SCALED_POWER;

        elevatorMtr.set(ControlMode.PercentOutput, mtrPower);
    }

    public void elevatorHoldingManual(double holdingEncPos)
    {
        elevatorMtr.set(ControlMode.Position, holdingEncPos);
    }

    public void elevatorSetToLowPos()
    {
        elevatorMtr.configAllowableClosedloopError(0, CLOSELOOP_ERROR_THRESHOLD_LOW);


        elevatorMtr.config_kP(0, ELEVATOR_KP_LOW);
        elevatorMtr.config_kI(0, ELEVATOR_KI_LOW);
        elevatorMtr.config_kD(0, ELEVATOR_KD_LOW);
        elevatorMtr.set(ControlMode.Position, POS_ENC_CNTS_LOW);
        targetPosition = POS_ENC_CNTS_LOW;
        elevatorInPosition = false;
    }

    public void elevatorSetToMidPosCone()
    {
        elevatorMtr.configAllowableClosedloopError(0, CLOSELOOP_ERROR_THRESHOLD_HIGH_MID);


        elevatorMtr.config_kP(0, ELEVATOR_KP_MID);
        elevatorMtr.config_kI(0, ELEVATOR_KI_MID);
        elevatorMtr.config_kD(0, ELEVATOR_KD_MID);
        elevatorMtr.set(ControlMode.Position, POS_ENC_CNTS_MID_CONE, DemandType.ArbitraryFeedForward, HOLDING_FEED_FORWARD);
        targetPosition = POS_ENC_CNTS_MID_CONE;
        elevatorInPosition = false;
    }

    public void elevatorSetToMidPosCube()
    {
        elevatorMtr.configAllowableClosedloopError(0, CLOSELOOP_ERROR_THRESHOLD_HIGH_MID);


        elevatorMtr.config_kP(0, ELEVATOR_KP_MID);
        elevatorMtr.config_kI(0, ELEVATOR_KI_MID);
        elevatorMtr.config_kD(0, ELEVATOR_KD_MID);
        elevatorMtr.set(ControlMode.Position, POS_ENC_CNTS_MID_CUBE, DemandType.ArbitraryFeedForward, HOLDING_FEED_FORWARD);
        targetPosition = POS_ENC_CNTS_MID_CUBE;
        elevatorInPosition = false;
    }

    public void elevatorSetToHighPos()
    {
        elevatorMtr.configAllowableClosedloopError(0, CLOSELOOP_ERROR_THRESHOLD_HIGH_MID);


        elevatorMtr.config_kP(0, ELEVATOR_KP_HIGH);
        elevatorMtr.config_kI(0, ELEVATOR_KI_HIGH);
        elevatorMtr.config_kD(0, ELEVATOR_KD_HIGH);
        elevatorMtr.set(ControlMode.Position, POS_ENC_CNTS_HIGH, DemandType.ArbitraryFeedForward, HOLDING_FEED_FORWARD);
        targetPosition = POS_ENC_CNTS_HIGH;
        elevatorInPosition = false;
    }

    public void elevatorSetToSinglePickup()
    {
        elevatorMtr.configAllowableClosedloopError(0, CLOSELOOP_ERROR_THRESHOLD_LOW);


        elevatorMtr.config_kP(0, ELEVATOR_KP_LOW);
        elevatorMtr.config_kI(0, ELEVATOR_KI_LOW);
        elevatorMtr.config_kD(0, ELEVATOR_KD_LOW);
        elevatorMtr.set(ControlMode.Position, 27739, DemandType.ArbitraryFeedForward, HOLDING_FEED_FORWARD);
        targetPosition = 27739;
        elevatorInPosition = false;
    }
    


    public void checkLimitSwitches()
    {
        if(elevatorMtr.getSensorCollection().isRevLimitSwitchClosed() == SWITCH_CLOSED)
        {
            elevatorMtr.setSelectedSensorPosition(POS_ENC_CNTS_LOW);
            lowSwitchState = true;
        }
        else
        {
            lowSwitchState = false;
        }

        if(elevatorMtr.getSensorCollection().isFwdLimitSwitchClosed() == SWITCH_CLOSED)
        {
            elevatorMtr.setSelectedSensorPosition(POS_ENC_CNTS_HIGH);
            highSwitchState = true;
        }
        else
        {
            highSwitchState = false;
        }
    }


    public double getElevatorEncoder()
    {
        return elevatorMtr.getSelectedSensorPosition();
    }

    public boolean isElevatorInPos()
    {
        return elevatorInPosition;
    }

    public static CatzElevatorSubsystem getInstance()
    {

        if(instance == null)
        {

            instance = new CatzElevatorSubsystem();

        }

        return instance;
    
    }

    
      
}
