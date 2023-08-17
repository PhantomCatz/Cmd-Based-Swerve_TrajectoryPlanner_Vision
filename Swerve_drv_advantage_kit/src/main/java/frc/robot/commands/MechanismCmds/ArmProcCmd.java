// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MechanismCmds;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.CatzConstants;
import frc.robot.Utils.CatzStateUtil;
import frc.robot.Utils.CatzStateUtil.ArmState;
import frc.robot.subsystems.Arm.CatzArmSubsystem;
import frc.robot.subsystems.Elevator.CatzElevatorSubsystem;


public class ArmProcCmd extends CommandBase {
  private CatzArmSubsystem arm = CatzArmSubsystem.getInstance();
  private CatzElevatorSubsystem elevator = CatzElevatorSubsystem.getInstance();
  private CatzStateUtil.ArmState currentArmState;
  private CatzStateUtil.SetMechanismState currentMechState;
  private boolean armExtend;
  private boolean armRetract;

  private boolean armAscent;

  private final double EXTEND_PWR  = 0.2;
  private final double RETRACT_PWR = -0.2;


  private final double MANUAL_CONTROL_PWR_OFF = 0.0;
  private final double HIGH_EXTEND_THRESHOLD_ELEVATOR = 73000.0;

  private double targetPosition = -999.0;
  private double currentPosition = -999.0;
  private double positionError = -999.0;

  private final double ARM_POS_ERROR_THRESHOLD = 2700.0; //0.5 inches    previously 500 enc counts

  private final double NO_TARGET_POSITION = -999999.0;

  private boolean armInPosition = false;

  private int numConsectSamples = 0;

  /** Creates a new ArmCmd. */
  public ArmProcCmd(CatzStateUtil.ArmState currentArmState, 
                CatzStateUtil.SetMechanismState currentMechState, 
                boolean armExtend, boolean armRetract) {

  this.currentArmState = currentArmState;
  this.currentMechState = currentMechState;
  this.armExtend = armExtend;
  this.armRetract = armRetract;
  addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    armAscent = false;
    armInPosition = false;
    if(currentArmState == ArmState.SET_STATE)
    {
      switch(currentMechState)
      {
          case STOW:
          case SCORE_MID :
              arm.armSetRetractPos();
              targetPosition = CatzConstants.POS_ENC_CNTS_EXTEND;
              break;

          case PICKUP_GROUND :
          case PICKUP_SINGLE :
          case SCORE_LOW :
              arm.armSetPickupPos();
              targetPosition = CatzConstants.POS_ENC_CNTS_PICKUP;
              break;

          case SCORE_HIGH :
                armAscent = true;
                targetPosition = CatzConstants.POS_ENC_CNTS_RETRACT;
              break;

          default:
              break;
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    if(currentArmState == ArmState.MANUAL)
    {
      if(armExtend == true)
      {
          arm.setArmPwr(EXTEND_PWR);
      }
      else if(armRetract == true)
      {
          arm.setArmPwr(RETRACT_PWR);
      }
      else if(arm.isArmControlModePercentOutput())
      {
          arm.setArmPwr(MANUAL_CONTROL_PWR_OFF);
      }
    }

    if((armAscent == true) && (elevator.getElevatorEncoder() >= HIGH_EXTEND_THRESHOLD_ELEVATOR))
    {
      arm.armSetFullExtendPos();
    }

    currentPosition = arm.getArmEncoder();
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

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    currentArmState = CatzStateUtil.ArmState.FINISHED;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if(armInPosition == true && currentArmState == ArmState.SET_STATE)
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}
