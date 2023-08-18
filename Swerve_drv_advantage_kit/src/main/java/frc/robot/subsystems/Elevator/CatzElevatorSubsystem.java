// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
//import frc.robot.Robot.mechMode;
import frc.robot.Utils.CatzStateUtil;
import frc.robot.Utils.CatzStateUtil.ElevatorState;
import frc.robot.Utils.CatzStateUtil.GamePieceState;
import frc.robot.subsystems.Arm.CatzArmSubsystem;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;





public class CatzElevatorSubsystem extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged  inputs = new ElevatorIOInputsAutoLogged();

  private static CatzElevatorSubsystem instance;


 //limit switch

  private final int SWITCH_CLOSED = 1;

  private boolean lowSwitchState  = false;
  private boolean highSwitchState = false;



  private final boolean LIMIT_SWITCH_IGNORED   = false;
  private final boolean LIMIT_SWITCH_MONITORED = true;  // limit switches will shut off the motor


  private final double MANUAL_CONTROL_DEADBAND = 0.1;
  private final double MANUAL_HOLD_STEP_SIZE = 10000.0; //5000.0;
  private final double ARM_ENCODER_THRESHOLD = 35000.0;

  private double manualHoldTargetPos;
  private boolean elevatorDescent;
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
            io = new ElevatorIOSim();
            break;
        default:
            io = new ElevatorIOReal() {};
            break;
    }


  }

  @Override
  public void periodic() 
  {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Elevator", inputs);
    
    checkLimitSwitches();
    // This method will be called once per scheduler run
  }


    public void elevatorManual(double pwr)
    {
        double mtrPower;

        mtrPower = pwr * CatzConstants.ELEVATOR_MAX_MANUAL_SCALED_POWER;

        io.elevatorManualIO(mtrPower);
    }

    public void elevatorHoldingManual(double holdingEncPos)
    {
        io.elevatorMtrSetPosIO(holdingEncPos);
    }

    public void elevatorSetToLowPos()
    {
        io.configAllowableClosedloopErrorIO(0, CatzConstants.ELEVATOR_CLOSELOOP_ERROR_THRESHOLD_LOW);


        io.elevatorConfig_kPIO(0, CatzConstants.ELEVATOR_KP_LOW);
        io.elevatorConfig_kIIO(0, CatzConstants.ELEVATOR_KI_LOW);
        io.elevatorConfig_kDIO(0, CatzConstants.ELEVATOR_KD_LOW);
        io.elevatorMtrSetPosIO(CatzConstants.ELEVATOR_POS_ENC_CNTS_LOW);
    }

    public void elevatorSetToMidPosCone()
    {
        io.configAllowableClosedloopErrorIO(0, CatzConstants.ELEVATOR_CLOSELOOP_ERROR_THRESHOLD_HIGH_MID);


        io.elevatorConfig_kPIO(0, CatzConstants.ELEVATOR_KP_MID);
        io.elevatorConfig_kIIO(0, CatzConstants.ELEVATOR_KI_MID);
        io.elevatorConfig_kDIO(0, CatzConstants.ELEVATOR_KD_MID);
        io.elevatorMtrSetPosIO(CatzConstants.ELEVATOR_POS_ENC_CNTS_MID_CONE);
    }

    public void elevatorSetToMidPosCube()
    {
        io.configAllowableClosedloopErrorIO(0, CatzConstants.ELEVATOR_CLOSELOOP_ERROR_THRESHOLD_HIGH_MID);


        io.elevatorConfig_kPIO(0, CatzConstants.ELEVATOR_KP_MID);
        io.elevatorConfig_kIIO(0, CatzConstants.ELEVATOR_KI_MID);
        io.elevatorConfig_kDIO(0, CatzConstants.ELEVATOR_KD_MID);
        io.elevatorMtrSetPosIO(CatzConstants.ELEVATOR_POS_ENC_CNTS_MID_CUBE);
    }

    public void elevatorSetToHighPos()
    {
        io.configAllowableClosedloopErrorIO(0, CatzConstants.ELEVATOR_CLOSELOOP_ERROR_THRESHOLD_HIGH_MID);


        io.elevatorConfig_kPIO(0, CatzConstants.ELEVATOR_KP_HIGH);
        io.elevatorConfig_kIIO(0, CatzConstants.ELEVATOR_KI_HIGH);
        io.elevatorConfig_kDIO(0, CatzConstants.ELEVATOR_KD_HIGH);
        io.elevatorMtrSetPosIO(CatzConstants.ELEVATOR_POS_ENC_CNTS_HIGH);
    }

    public void elevatorSetToSinglePickup()
    {
        io.configAllowableClosedloopErrorIO(0, CatzConstants.ELEVATOR_CLOSELOOP_ERROR_THRESHOLD_LOW);


        io.elevatorConfig_kPIO(0, CatzConstants.ELEVATOR_KP_LOW);
        io.elevatorConfig_kIIO(0, CatzConstants.ELEVATOR_KI_LOW);
        io.elevatorConfig_kDIO(0, CatzConstants.ELEVATOR_KD_LOW);
        io.elevatorMtrSetPosIO(CatzConstants.ELEVATOR_POS_ENC_CNTS_SINGLE_PICKUP);
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
            io.setSelectedSensorPositionIO(CatzConstants.ELEVATOR_POS_ENC_CNTS_LOW);
            lowSwitchState = true;
        }
        else
        {
            lowSwitchState = false;
        }

        if(inputs.isFwdLimitSwitchClosed)
        {
            io.setSelectedSensorPositionIO(CatzConstants.ELEVATOR_POS_ENC_CNTS_HIGH);
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
