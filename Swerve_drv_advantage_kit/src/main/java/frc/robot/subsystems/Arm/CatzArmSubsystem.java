// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;


import frc.robot.*;
//import frc.robot.Robot.mechMode;

public class CatzArmSubsystem extends SubsystemBase 
{
  /** Creates a new CatzArmSubsystem. */
  private final ArmIO io;
  private final ArmIOInputsAutoLogged  inputs = new ArmIOInputsAutoLogged();

  private static CatzArmSubsystem instance;



  private final boolean LIMIT_SWITCH_IGNORED = false;
  private final boolean LIMIT_SWITCH_MONITORED = true;


  private boolean extendSwitchState = false;

  private int SWITCH_CLOSED = 1;


  private final double ARM_CLOSELOOP_ERROR = 3000;





  private CatzArmSubsystem() 
  {
    switch(CatzConstants.currentMode)
    {
        case REAL:
            io = new ArmIOReal();
            break;
        case SIM :
            io = new ArmIOSim();
            break;
        default:
            io = new ArmIOReal() {};
            break;
    }

  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Arm", inputs);

    checkLimitSwitches();
  }

  
  public void checkLimitSwitches() {

        if(inputs.isRevLimitSwitchClosed) 
        {
            io.setSelectedSensorPositionIO(CatzConstants.POS_ENC_CNTS_RETRACT);
            extendSwitchState = true;
        }
        else 
        {
            extendSwitchState = false;
        }
    }


    public void setArmPwr(double pwr)
    {        
        io.setArmPwrIO(pwr);
    }

    public double getArmEncoder()
    {
        return inputs.armMotorEncoder;
    }

    public void armSetFullExtendPos()
    {
        io.armSetFullExtendPosIO();
    }

    public void armSetRetractPos()
    {
        io.armSetRetractPosIO();
    }

    public void armSetPickupPos()
    {
        io.armSetPickupPosIO();
    }

    public boolean isArmControlModePercentOutput()
    {
        return inputs.currentArmControlMode;
    }

    public static CatzArmSubsystem getInstance()
    {

        if(instance == null)
        {

            instance = new CatzArmSubsystem();

        }

        return instance;
    
    }
}
