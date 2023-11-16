// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.Utils.CatzManipulatorPositions;
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

  static CatzArmSubsystem arm = CatzArmSubsystem.getInstance();

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

  private double sharedElevatorEncoderUpdate;

  private CatzManipulatorPositions targetPose;




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

  //The periodic method will be used for any non stateset hardware implementation
  @Override
  public void periodic() 
  {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Elevator", inputs);
    sharedElevatorEncoderUpdate = inputs.elevatorEncoderCnts;
    
    checkLimitSwitches();
    // This method will be called once per scheduler run

    if(DriverStation.isDisabled())
    {
        io.elevatorManualIO(0.0);
        elevatorControlState = null;
        elevatorSetStateUpdate = null;
    }
    else if(targetPose != null)
    {
        io.elevatorMtrSetPosIO(targetPose.getElevatorPosEnc());
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
    elevatorHoldingManual(elevatorPwr);
    
    if(isFullManualEnabled)
    {
      elevatorControlState = ElevatorControlState.FULLMANUAL;
    }
    else
    {
      elevatorControlState = ElevatorControlState.SEMIMANUAL;
    }

  }

  public void cmdUpdateElevator(CatzManipulatorPositions targetPosition)
  {
    elevatorControlState = ElevatorControlState.AUTO;
    this.targetPose = targetPosition;
    //TBD add motor configs with if statments
  }



    private void elevatorManual(double pwr)
    {
        double mtrPower;

        mtrPower = pwr * CatzConstants.ElevatorConstants.ELEVATOR_MAX_MANUAL_SCALED_POWER;

        io.elevatorManualIO(mtrPower);
    }

    public void elevatorHoldingManual(double holdingEncPos)
    {
        io.elevatorMtrSetPosIO(holdingEncPos);
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
      //System.out.println(sharedElevatorEncoderUpdate);
        return sharedElevatorEncoderUpdate;
    }

    public ElevatorControlState getElevatorControlState()
    {
        return elevatorControlState;
    }

    //Singleton implementation for instatiating subssytems(Every refrence to this method should be static)
    public static CatzElevatorSubsystem getInstance()
    {
        if(instance == null)
        {
            instance = new CatzElevatorSubsystem();
        }
        return instance;
    }
  }
