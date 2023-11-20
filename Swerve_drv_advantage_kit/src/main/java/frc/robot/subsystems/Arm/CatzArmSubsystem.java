// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;


import frc.robot.*;
import frc.robot.CatzConstants.ArmConstants;
import frc.robot.CatzConstants.ElevatorConstants;
import frc.robot.Utils.CatzManipulatorPositions;
import frc.robot.Utils.CatzSharedDataUtil;
import frc.robot.Utils.CatzAbstractStateUtil;
//import frc.robot.Robot.mechMode;
import frc.robot.subsystems.Elevator.CatzElevatorSubsystem;

public class CatzArmSubsystem extends SubsystemBase 
{
  /** Creates a new CatzArmSubsystem. */
  private final ArmIO io;
  private final ArmIOInputsAutoLogged  inputs = new ArmIOInputsAutoLogged();

  private static CatzArmSubsystem instance;

  static CatzElevatorSubsystem elevator = CatzElevatorSubsystem.getInstance();

  private final double HIGH_EXTEND_THRESHOLD_ELEVATOR = 73000.0;
  private final double ARM_POS_ERROR_THRESHOLD = 2700.0; //0.5 inches    previously 500 enc counts

  private boolean extendSwitchState = false;

  private final double ARM_CLOSELOOP_ERROR = 3000;



  private final double NO_TARGET_POSITION = -999999.0;

  private boolean armInPosition = false;

  private int numConsectSamples = 0;

  private double m_armPwr = -999.0;
  private boolean m_isArmInExtension = false;

  private CatzManipulatorPositions m_targetPose;


  private CatzArmSubsystem() {
    switch(CatzConstants.currentMode) {
        case REAL: io = new ArmIOReal();
            break;
        case SIM : io = null;// new ArmIOSim();
            break;
        default: io = new ArmIOReal() {};
            break;
    }
  }

  @Override
  public void periodic() {
    //perform input updates
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Arm", inputs);
    checkLimitSwitches();

    //arm logic implementation requiring a loop
    if(DriverStation.isDisabled()) {
        io.setArmPwrIO(0.0);
    }
    else if(m_targetPose != null) {
        if(m_isArmInExtension) {
            if(CatzSharedDataUtil.sharedElevatorEncCnts > HIGH_EXTEND_THRESHOLD_ELEVATOR)
            io.setArmPosEncIO(m_targetPose.getArmPosEnc());
        }
        else {
            io.setArmPosEncIO(m_targetPose.getArmPosEnc());
        }
    }
    else {
        io.setArmPwrIO(m_armPwr);
    }

    //checking if arm has reached position
    double currentPosition = inputs.armMotorEncoder;
    double positionError = currentPosition - m_targetPose.getArmPosEnc();
    if  ((Math.abs(positionError) <= ARM_POS_ERROR_THRESHOLD)) {
        numConsectSamples++;
            if(numConsectSamples >= 10) {   
                CatzSharedDataUtil.sharedArmInPos = true;
            }
    }
    else {
        numConsectSamples = 0;
        CatzSharedDataUtil.sharedArmInPos = false;
    }
  }
    //updates the arm statemachine to auto and does auto cmds
    public void cmdUpdateArm(CatzManipulatorPositions targetPose)
    {
      this.m_targetPose = targetPose;
      if(m_targetPose.getArmPosEnc() > ArmConstants.POS_ENC_CNTS_PICKUP) {
          m_isArmInExtension = true;
      }
      else {
          m_isArmInExtension = false;
      }
    }
  
    public void setArmPwr(double pwr) {        
        this.m_armPwr = pwr;
        this.m_targetPose = null;
    }

    /*----------------------------------------------------------------------------------------------
    *
    *  Utilities - 
    *
    *---------------------------------------------------------------------------------------------*/

    public void checkLimitSwitches() {
    //recalibrating arm position with the rev limit switch is triggered
    if(inputs.isRevLimitSwitchClosed) {
        io.setSelectedSensorPositionIO(CatzConstants.ArmConstants.POS_ENC_CNTS_RETRACT);
        extendSwitchState = true;
    }
    else {
        extendSwitchState = false;
    }
  }

    //Singleton implementation for instatiating subssytems(Every refrence to this method should be static)
    public static CatzArmSubsystem getInstance() {
        if(instance == null) {
            instance = new CatzArmSubsystem();
        }
        return instance;
    }
}
