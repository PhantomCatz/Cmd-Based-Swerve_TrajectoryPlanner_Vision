// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;


import frc.robot.*;
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

  private final boolean LIMIT_SWITCH_IGNORED = false;
  private final boolean LIMIT_SWITCH_MONITORED = true;


  private boolean extendSwitchState = false;

  private int SWITCH_CLOSED = 1;


  private final double ARM_CLOSELOOP_ERROR = 3000;

  private double targetPosition = -999.0;
  private double currentPosition = -999.0;
  private double positionError = -999.0;

  private final double ARM_POS_ERROR_THRESHOLD = 2700.0; //0.5 inches    previously 500 enc counts

  private final double NO_TARGET_POSITION = -999999.0;

  private boolean armInPosition = false;

  private int numConsectSamples = 0;

  private double sharedArmEncoderUpdate;
  private boolean sharedArmControlModeUpdate;

  private boolean armAscent;
  private double armPower;


  private CatzArmSubsystem() 
  {
    switch(CatzConstants.currentMode)
    {
        case REAL:
            io = new ArmIOReal();
            break;
        case SIM :
            io = null;// new ArmIOSim();
            break;
        default:
            io = new ArmIOReal() {};
            break;
    }
  }

  private static ArmAutoState armSetState = null;
  public static enum ArmAutoState {
    
    EXTEND,
    RETRACT,
    PICKUP,
  }  
  private static ArmControlState armControlState = null;
  public static enum ArmControlState {
    
    AUTO,
    FULLMANUAL,
  } 


  @Override
  public void periodic() 
  {
    //perform input updates
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Arm", inputs);
    checkLimitSwitches();

    //shoving input variables into class member variables for transfering to different java files
    sharedArmEncoderUpdate = inputs.armMotorEncoder;
    sharedArmControlModeUpdate = inputs.currentArmControlMode; 

    //arm logic implementation requiring a loop
    if(DriverStation.isDisabled())
    {
        io.setArmPwrIO(0.0);
        armControlState = null;
        armSetState = null;
    }
    else if(armControlState == ArmControlState.FULLMANUAL)
    {
        io.setArmPwrIO(armPower);
    }
    else if(armControlState == ArmControlState.AUTO)
    {
        if((armAscent == true) && (elevator.getElevatorEncoder() >= HIGH_EXTEND_THRESHOLD_ELEVATOR))
        {
            io.armSetFullExtendPosIO();
        }
    }

    //checking if arm has reached position
    currentPosition = getArmEncoder();
    positionError = currentPosition - targetPosition;
    if  ((Math.abs(positionError) <= ARM_POS_ERROR_THRESHOLD) && targetPosition != NO_TARGET_POSITION) 
    {
        targetPosition = NO_TARGET_POSITION;
        numConsectSamples++;
            if(numConsectSamples >= 10) 
            {   
                armInPosition = true;
            }
    }
    else 
    {
        numConsectSamples = 0;
    }
  }

  public void checkLimitSwitches() 
  {
    if(inputs.isRevLimitSwitchClosed) 
    {
        io.setSelectedSensorPositionIO(CatzConstants.ArmConstants.POS_ENC_CNTS_RETRACT);
        extendSwitchState = true;
    }
    else 
    {
        extendSwitchState = false;
    }
  }

  //updates the arm statemachine to auto and does auto cmds
  public void cmdUpdateArm(ArmAutoState state)
  {
    armControlState = ArmControlState.AUTO;
    armSetState = state;   

    if(armSetState != ArmAutoState.EXTEND)
    {
        armAscent = false;
    }

    switch(armSetState)
    {
        case EXTEND:
        armAscent = true;
        break;


        case RETRACT:
        io.armSetRetractPosIO();
        break;


        case PICKUP:
        io.armSetPickupPosIO();

        break;
    }
  }

  public void setArmPwr(double pwr)
  {        
      io.setArmPwrIO(pwr);
      armControlState = ArmControlState.FULLMANUAL;
  }

    public double getArmEncoder()
    {
        return sharedArmEncoderUpdate;
    }

    public boolean isArmControlModePercentOutput()
    {
        return sharedArmControlModeUpdate;
    }

    public ArmControlState getArmControlState()
    {
        return armControlState;
    }

    //Singleton implementation for instatiating subssytems(Every refrence to this method should be static)
    public static CatzArmSubsystem getInstance()
    {
        if(instance == null)
        {
            instance = new CatzArmSubsystem();
        }
        return instance;
    }
}
