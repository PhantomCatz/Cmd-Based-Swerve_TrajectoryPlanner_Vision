// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.Utils.CatzStateUtil;
import frc.robot.Utils.CatzStateUtil.GamePieceState;
//import frc.robot.Robot.mechMode;
import frc.robot.subsystems.Arm.CatzArmSubsystem;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;


public class CatzElevatorSubsystem extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged  inputs = new ElevatorIOInputsAutoLogged();

  private static CatzElevatorSubsystem instance;

  CatzArmSubsystem arm = CatzArmSubsystem.getInstance();

  private final double ARM_ENCODER_THRESHOLD = 35000.0;
  private final double MANUAL_HOLD_STEP_SIZE = 10000.0; //5000.0;

 //limit switch

  private final int SWITCH_CLOSED = 1;

  private boolean lowSwitchState  = false;
  private boolean highSwitchState = false;



  private final boolean LIMIT_SWITCH_IGNORED   = false;
  private final boolean LIMIT_SWITCH_MONITORED = true;  // limit switches will shut off the motor

  private boolean elevatorDescent = false;
  private double manualHoldTargetPos;
  private double elevatorPwr;
  private double targetPosition = -999.0;
  private double currentPosition = -999.0;
  private double positionError = -999.0;

  private final double ELEVATOR_POS_ERROR_THRESHOLD = 1000.0; //0.424 inches

  private final double NO_TARGET_POSITION = -999999.0;

  private boolean elevatorInPosition = false;

  private int numConsectSamples = 0;




  /** Creates a new CatzElevatorSubsystem. */
  private CatzElevatorSubsystem() 
  {
    switch(CatzConstants.currentMode)
    {
        case REAL:
            io = new ElevatorIOReal();
            break;
        case SIM :
            io = null; //new ElevatorIOSim();
            break;
        default:
            io = new ElevatorIOReal() {};
            break;
    }
  }

  private static ElevatorAutoState elevatorSetStateUpdate = null;
  public static enum ElevatorAutoState {

    HIGH,
    MIDCONE,
    MIDCUBE,
    LOW,
  }  

  private static ElevatorControlState elevatorControlState = null;
  public static enum ElevatorControlState {
    
    AUTO,
    SEMIMANUAL,
    FULLMANUAL,
  } 

  @Override
  public void periodic() 
  {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Elevator", inputs);
    
    checkLimitSwitches();
    // This method will be called once per scheduler run

    if(DriverStation.isDisabled())
    {
        io.elevatorManualIO(0.0);
        elevatorControlState = null;
        elevatorSetStateUpdate = null;
    }
    else if(elevatorControlState == ElevatorControlState.FULLMANUAL)
    {
      elevatorManual(elevatorPwr);
    }
    else if(elevatorControlState == ElevatorControlState.SEMIMANUAL)
    {
      manualHoldTargetPos = getElevatorEncoder();
      manualHoldTargetPos = manualHoldTargetPos + (elevatorPwr * MANUAL_HOLD_STEP_SIZE);
      elevatorHoldingManual(manualHoldTargetPos);
    }
    else
    {
      if((elevatorDescent == true) && (arm.getArmEncoder() <= ARM_ENCODER_THRESHOLD))
      {
        elevatorSetToLowPos();
      }
    }

    //checking elevator is in position
    currentPosition = getElevatorEncoder();
    positionError = currentPosition - targetPosition;
    if  ((Math.abs(positionError) <= ELEVATOR_POS_ERROR_THRESHOLD) && targetPosition != NO_TARGET_POSITION) 
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

  public void elevatorManualCmd(double elevatorPwr, boolean isFullManualEnabled)
  {
    this.elevatorPwr = elevatorPwr;
    
    if(isFullManualEnabled)
    {
      elevatorControlState = ElevatorControlState.FULLMANUAL;
    }
    else
    {
      elevatorControlState = ElevatorControlState.SEMIMANUAL;
    }

  }

  public void setElevatorAutoState(ElevatorAutoState state)
  {
    this.elevatorSetStateUpdate = state;
    elevatorControlState = ElevatorControlState.AUTO;

    if(elevatorSetStateUpdate != ElevatorAutoState.LOW)
    {
      elevatorDescent = false;
    }

    switch(elevatorSetStateUpdate)
    {
      case HIGH:
      elevatorSetToHighPos();
      break;

      case MIDCONE:
      elevatorSetToMidPosCone();
      break;

      case MIDCUBE:
      elevatorSetToMidPosCube();
      break;

      case LOW:
      elevatorDescent = true;
      break;
    }
  }



    public void elevatorManual(double pwr)
    {
        double mtrPower;

        mtrPower = pwr * CatzConstants.ElevatorConstants.ELEVATOR_MAX_MANUAL_SCALED_POWER;

        io.elevatorManualIO(mtrPower);
    }

    public void elevatorHoldingManual(double holdingEncPos)
    {
        io.elevatorMtrSetPosIO(holdingEncPos);
    }

    private void elevatorSetToLowPos()
    {
        io.configAllowableClosedloopErrorIO(0, CatzConstants.ElevatorConstants.ELEVATOR_CLOSELOOP_ERROR_THRESHOLD_LOW);


        io.elevatorConfig_kPIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KP_LOW);
        io.elevatorConfig_kIIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KI_LOW);
        io.elevatorConfig_kDIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KD_LOW);
        io.elevatorMtrSetPosIO(CatzConstants.ElevatorConstants.ELEVATOR_POS_ENC_CNTS_LOW);
    }

    private void elevatorSetToMidPosCone()
    {
        io.configAllowableClosedloopErrorIO(0, CatzConstants.ElevatorConstants.ELEVATOR_CLOSELOOP_ERROR_THRESHOLD_HIGH_MID);


        io.elevatorConfig_kPIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KP_MID);
        io.elevatorConfig_kIIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KI_MID);
        io.elevatorConfig_kDIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KD_MID);
        io.elevatorMtrSetPosIO(CatzConstants.ElevatorConstants.ELEVATOR_POS_ENC_CNTS_MID_CONE);
    }

    private void elevatorSetToMidPosCube()
    {
        io.configAllowableClosedloopErrorIO(0, CatzConstants.ElevatorConstants.ELEVATOR_CLOSELOOP_ERROR_THRESHOLD_HIGH_MID);


        io.elevatorConfig_kPIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KP_MID);
        io.elevatorConfig_kIIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KI_MID);
        io.elevatorConfig_kDIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KD_MID);
        io.elevatorMtrSetPosIO(CatzConstants.ElevatorConstants.ELEVATOR_POS_ENC_CNTS_MID_CUBE);
    }

    private void elevatorSetToHighPos()
    {
        io.configAllowableClosedloopErrorIO(0, CatzConstants.ElevatorConstants.ELEVATOR_CLOSELOOP_ERROR_THRESHOLD_HIGH_MID);


        io.elevatorConfig_kPIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KP_HIGH);
        io.elevatorConfig_kIIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KI_HIGH);
        io.elevatorConfig_kDIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KD_HIGH);
        io.elevatorMtrSetPosIO(CatzConstants.ElevatorConstants.ELEVATOR_POS_ENC_CNTS_HIGH);
    }

    private void elevatorSetToSinglePickup()
    {
        io.configAllowableClosedloopErrorIO(0, CatzConstants.ElevatorConstants.ELEVATOR_CLOSELOOP_ERROR_THRESHOLD_LOW);


        io.elevatorConfig_kPIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KP_LOW);
        io.elevatorConfig_kIIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KI_LOW);
        io.elevatorConfig_kDIO(0, CatzConstants.ElevatorConstants.ELEVATOR_KD_LOW);
        io.elevatorMtrSetPosIO(CatzConstants.ElevatorConstants.ELEVATOR_POS_ENC_CNTS_SINGLE_PICKUP);
    }

    

    /*----------------------------------------------------------------------------------------------
    *
    *  Utilities - 
    *
    *---------------------------------------------------------------------------------------------*/
    public void checkLimitSwitches()
    {
        if(inputs.isRevLimitSwitchClosed)
        {
            io.setSelectedSensorPositionIO(CatzConstants.ElevatorConstants.ELEVATOR_POS_ENC_CNTS_LOW);
            lowSwitchState = true;
        }
        else
        {
            lowSwitchState = false;
        }

        if(inputs.isFwdLimitSwitchClosed)
        {
            io.setSelectedSensorPositionIO(CatzConstants.ElevatorConstants.ELEVATOR_POS_ENC_CNTS_HIGH);
            highSwitchState = true;
        }
        else
        {
            highSwitchState = false;
        }
    }


    public double getElevatorEncoder()
    {
        return inputs.elevatorEncoderCnts;
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
