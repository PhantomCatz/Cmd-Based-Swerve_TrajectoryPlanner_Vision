// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MechanismCmds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils.CatzStateUtil;
import frc.robot.Utils.CatzStateUtil.GamePieceState;
import frc.robot.Utils.CatzStateUtil.IntakeState;
import frc.robot.subsystems.Intake.CatzIntakeSubsystem;

public class IntakeCmd extends CommandBase {
  CatzIntakeSubsystem intake;
  CatzStateUtil.IntakeState currentIntakeState;
  CatzStateUtil.MechanismState currentMechanismState;
  Supplier<Double> supplierWristPwr;
  Supplier<Boolean> supplierManualMode;

  private final double SOFT_LIMIT_FORWARD = 0.0; //4876  + WRIST_ABS_ENC_OFFSET;  //3887
  private final double SOFT_LIMIT_REVERSE = -8900.0; //-798.0 + WRIST_ABS_ENC_OFFSET; //-1787     //TBD
  private final double MANUAL_HOLD_STEP_SIZE = 1.5;       
  private final double WRIST_MAX_PWR = 0.3;



  private boolean  pidEnable = false;
  private boolean intakeInPosition = false;

  private double   targetPower = 0.0;
  private double   targetPosDegCmd;
  /** Creates a new IntakeCmd. */
  public IntakeCmd(CatzIntakeSubsystem intake,
                   CatzStateUtil.IntakeState currentIntakeState,
                   CatzStateUtil.MechanismState currentMechState,
                   Supplier<Double> wristPwr, Supplier<Boolean> supplierManualMode) {
    this.intake = intake;
    this.currentMechanismState = currentMechState;
    this.supplierWristPwr = wristPwr;
    this.supplierManualMode = supplierManualMode;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    if(currentIntakeState == IntakeState.SET_STATE)
    {
      pidEnable = true;
      switch(currentMechanismState)
      {
          case  STOW:
            targetPosDegCmd = intake.STOW_ENC_POS;
              break;
              
          case PICKUP_GROUND :
            if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
              {
                targetPosDegCmd = intake.INTAKE_CUBE_ENC_POS;
              }
            else
              {
                targetPosDegCmd = intake.INTAKE_PICKUP_CONE_ENC_POS_GROUND;
              }
              break;

          case PICKUP_SINGLE :
            if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
              {

              }
            else
              {
                targetPosDegCmd = intake.INTAKE_PICKUP_CONE_ENC_POS_SINGLE;
              }
              break;
              
          case SCORE_LOW :
            if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
              {
                targetPosDegCmd = intake.SCORE_CUBE_ENC_POS;
              }
            else
              {
                targetPosDegCmd = intake.SCORE_CONE_LOW_ENC_POS;
              }
              break;

          case SCORE_MID :
            if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
              {
                targetPosDegCmd = intake.SCORE_CUBE_ENC_POS;
              }
            else
              {
                targetPosDegCmd = intake.SCORE_CONE_MID_ENC_POS;
              }
              break;
              
          case SCORE_HIGH :
            if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
              {
                targetPosDegCmd = intake.SCORE_CUBE_ENC_POS;
              }
            else
              {
                targetPosDegCmd = intake.SCORE_CONE_HIGH_ENC_POS; 
              }
              break;

          default:
              //TBD
              break;
      }
    }
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    boolean manualMode = supplierManualMode.get();
    double wristPwr =     supplierWristPwr.get();
    if(currentIntakeState == IntakeState.MANUAL)
    {
      if(manualMode)
      {                
        pidEnable = false;
      }

    if(Math.abs(wristPwr) >= 0.1)//if we are apply wrist power manually
    {
        if (pidEnable == true)//check if in manual holding state
        {

            if(wristPwr > 0)
            {
              targetPosDegCmd = Math.min((intake.targetPositionDeg + wristPwr * MANUAL_HOLD_STEP_SIZE), SOFT_LIMIT_FORWARD);
            }
            else
            {
              targetPosDegCmd = Math.max((intake.targetPositionDeg + wristPwr * MANUAL_HOLD_STEP_SIZE), SOFT_LIMIT_REVERSE);
            }
            intake.prevCurrentPosition = -intake.prevCurrentPosition; //intialize for first time through thread loop, that checks stale position values
        }
        else //in full manual mode
        {
            targetPower = wristPwr * WRIST_MAX_PWR;    
            intake.wristSetPercentOuput(targetPower);
        }
    }
    else //Manual power is OFF
    {
        if(pidEnable == false)//if we are still in manual mode and want to hold intake in place
        {
            targetPower = 0.0;
            intake.wristSetPercentOuput(targetPower);
        }
    }


    if(pidEnable)
    {
      intake.intakePIDLoopFunction(targetPosDegCmd);
    }
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if(currentIntakeState == IntakeState.MANUAL)
    {
      return false;
    }
    else if(currentIntakeState == IntakeState.SET_STATE)
      if(intakeInPosition)
      {
        return true;
      }
      else
      {
        return false;
      }

    else
    {
      return false;
    }
  }
}

