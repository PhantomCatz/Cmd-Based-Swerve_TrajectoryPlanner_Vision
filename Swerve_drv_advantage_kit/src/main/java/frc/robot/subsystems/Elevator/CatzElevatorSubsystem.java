// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.ElevatorConstants;
import frc.robot.Utils.CatzManipulatorPositions;
import frc.robot.Utils.CatzSharedDataUtil;

import org.littletonrobotics.junction.Logger;


public class CatzElevatorSubsystem extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged  inputs = new ElevatorIOInputsAutoLogged();

  private static CatzElevatorSubsystem instance;

  private final double ARM_ENCODER_THRESHOLD = 35000.0;
  private final double ELEVATOR_POS_ERROR_THRESHOLD = 1000.0; //0.424 inches
  private final double NO_TARGET_POSITION = -999999.0;

 //limit switch
  private boolean m_lowSwitchState  = false;
  private boolean m_highSwitchState = false;

  private boolean m_elevatorDescent = false;
  private double m_elevatorPwr;

  private boolean m_elevatorInPosition = false;
  private int m_numConsectSamples = 0;

  //manipulator target pose object
  private CatzManipulatorPositions m_targetPose;

  /** Creates a new CatzElevatorSubsystem. */
  private CatzElevatorSubsystem() {
    switch(CatzConstants.currentMode) {
        case REAL:io = new ElevatorIOReal();
            break;
        case SIM :io = null; //new ElevatorIOSim();
            break;
        default: io = new ElevatorIOReal() {};
            break;
    }
  }


  //The periodic method will be used for any hardware calls to set motor power
  @Override
  public void periodic() {
    //updating inputs
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Elevator", inputs);  
    checkLimitSwitches();

    //determining whether to set power to motor
    if(DriverStation.isDisabled()) {
        io.elevatorManualIO(0.0);
    }
    else if(m_targetPose != null) {
      //watching if the elevator acting in a area where the arm position needs to be watched
      if(m_elevatorDescent) {
        if(CatzSharedDataUtil.sharedArmEncCnts < ARM_ENCODER_THRESHOLD) {
          io.elevatorMtrSetPosIO(m_targetPose.getElevatorPosEnc());
        }
      }
      else {
         io.elevatorMtrSetPosIO(m_targetPose.getElevatorPosEnc());
      }
    }
    else { //full manual
      m_elevatorPwr = m_elevatorPwr * CatzConstants.ElevatorConstants.ELEVATOR_MAX_MANUAL_SCALED_POWER;
      io.elevatorManualIO(m_elevatorPwr);
    }

    //checking elevator is in position
    double currentPosition = inputs.elevatorEncoderCnts;
    double positionError = currentPosition - m_targetPose.getElevatorPosEnc();
    if  ((Math.abs(positionError) <= ELEVATOR_POS_ERROR_THRESHOLD) && m_targetPose.getElevatorPosEnc() != NO_TARGET_POSITION) {
        m_numConsectSamples++;
            if(m_numConsectSamples >= 10) {   
                CatzSharedDataUtil.sharedElevatorInPos = true;
            }
    }
    else {
        m_numConsectSamples = 0;
        CatzSharedDataUtil.sharedElevatorInPos = false;
    }
  }

  public void elevatorFullManualCmd(double elevatorPwr) {
    this.m_elevatorPwr = elevatorPwr;
    m_targetPose = null;
  }

  public void cmdUpdateElevator(CatzManipulatorPositions targetPosition) {
    this.m_targetPose = targetPosition;

    //setting different pid values depending on going up or down
    if(targetPosition.getElevatorPosEnc() > inputs.elevatorEncoderCnts) {
      io.elevatorConfig_kPIO(0, ElevatorConstants.ELEVATOR_KP_HIGH);
      io.elevatorConfig_kIIO(0, ElevatorConstants.ELEVATOR_KI_HIGH);
      io.elevatorConfig_kDIO(0, ElevatorConstants.ELEVATOR_KD_HIGH);
    }
    else {
      io.elevatorConfig_kPIO(0, ElevatorConstants.ELEVATOR_KP_LOW);
      io.elevatorConfig_kIIO(0, ElevatorConstants.ELEVATOR_KI_LOW);
      io.elevatorConfig_kDIO(0, ElevatorConstants.ELEVATOR_KD_LOW);
    }

    //if the target position is lower than High elevator needs to be aware of the arm position
    if(targetPosition.getElevatorPosEnc() < ElevatorConstants.ELEVATOR_POS_ENC_CNTS_HIGH) {
      m_elevatorDescent = true;
    }
    else {
      m_elevatorDescent = false;
    }
  }

    /*----------------------------------------------------------------------------------------------
    *
    *  Utilities - 
    *
    *---------------------------------------------------------------------------------------------*/
    public void checkLimitSwitches()
    {
      //recalibrate position to low if the rev limit switch is triggered
      if(inputs.isRevLimitSwitchClosed) {
          io.setSelectedSensorPositionIO(CatzConstants.ElevatorConstants.ELEVATOR_POS_ENC_CNTS_LOW);
          m_lowSwitchState = true;
      }
      else {
          m_lowSwitchState = false;
      }
      //recalibrate position to high if the high limit switch is triggered
      if(inputs.isFwdLimitSwitchClosed) {
          io.setSelectedSensorPositionIO(CatzConstants.ElevatorConstants.ELEVATOR_POS_ENC_CNTS_HIGH);
          m_highSwitchState = true;
      }
      else {
          m_highSwitchState = false;
      }
    }

    //Singleton implementation for instatiating subssytems(Every refrence to this method should be static)
    public static CatzElevatorSubsystem getInstance() {
        if(instance == null) {
            instance = new CatzElevatorSubsystem();
        }
        return instance;
    }
  }
