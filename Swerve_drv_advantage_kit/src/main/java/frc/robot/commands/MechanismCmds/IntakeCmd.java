// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MechanismCmds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.CatzConstants;
import frc.robot.Utils.CatzStateUtil;
import frc.robot.Utils.CatzStateUtil.GamePieceState;
import frc.robot.Utils.CatzStateUtil.IntakeState;
import frc.robot.subsystems.Intake.CatzIntakeSubsystem;

public class IntakeCmd extends CommandBase {
  CatzIntakeSubsystem intake = CatzIntakeSubsystem.getInstance();
  CatzStateUtil.IntakeState currentIntakeState;
  CatzStateUtil.MechanismState currentMechanismState;
  Supplier<Double> supplierWristPwr;
  Supplier<Boolean> supplierManualMode;

  private final double SOFT_LIMIT_FORWARD = 0.0; //4876  + WRIST_ABS_ENC_OFFSET;  //3887
  private final double SOFT_LIMIT_REVERSE = -8900.0; //-798.0 + WRIST_ABS_ENC_OFFSET; //-1787     //TBD
  private final double MANUAL_HOLD_STEP_SIZE = 1.5;       
  private final double WRIST_MAX_PWR = 0.3;



  private boolean  pidEnableCmd = false;

  private double   targetPowerCmd = 0.0;
  private double   targetPosDegCmd;



  private int numConsectSamples = 0;
  /** Creates a new IntakeCmd. */
  public IntakeCmd(CatzStateUtil.IntakeState currentIntakeState,
                   CatzStateUtil.MechanismState currentMechState,
                   Supplier<Double> wristPwr, Supplier<Boolean> supplierManualMode) {

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
      pidEnableCmd = true;
      intake.setisIntakeInPos(false);
      switch(currentMechanismState)
      {
          case  STOW:
            targetPosDegCmd = CatzConstants.STOW_ENC_POS;
              break;
              
          case PICKUP_GROUND :
            if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
              {
                targetPosDegCmd = CatzConstants.INTAKE_CUBE_ENC_POS;
              }
            else
              {
                targetPosDegCmd = CatzConstants.INTAKE_PICKUP_CONE_ENC_POS_GROUND;
              }
              break;

          case PICKUP_SINGLE :
            if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
              {

              }
            else
              {
                targetPosDegCmd = CatzConstants.INTAKE_PICKUP_CONE_ENC_POS_SINGLE;
              }
              break;
              
          case SCORE_LOW :
            if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
              {
                targetPosDegCmd = CatzConstants.SCORE_CUBE_ENC_POS;
              }
            else
              {
                targetPosDegCmd = CatzConstants.SCORE_CONE_LOW_ENC_POS;
              }
              break;

          case SCORE_MID :
            if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
              {
                targetPosDegCmd = CatzConstants.SCORE_CUBE_ENC_POS;
              }
            else
              {
                targetPosDegCmd = CatzConstants.SCORE_CONE_MID_ENC_POS;
              }
              break;
              
          case SCORE_HIGH :
            if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
              {
                targetPosDegCmd = CatzConstants.SCORE_CUBE_ENC_POS;
              }
            else
              {
                targetPosDegCmd = CatzConstants.SCORE_CONE_HIGH_ENC_POS; 
              }
              break;

          default:
              //TBD
              break;
      }
      intake.setTargetPositionDegState(targetPosDegCmd);
    }
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    boolean manualModeCmd = supplierManualMode.get();
    double wristPwrCmd   = supplierWristPwr.get();
    if(currentIntakeState == IntakeState.MANUAL)
    {
      if(manualModeCmd)
      {                
        pidEnableCmd = false;
      }

      if(Math.abs(wristPwrCmd) >= 0.1)//if we are apply wrist power manually
      {
          if (pidEnableCmd == true)//check if in manual holding state
          {
            intake.manualHoldingFunction(wristPwrCmd);
          }
          else //in full manual mode
          {
              targetPowerCmd = wristPwrCmd * WRIST_MAX_PWR;    
              intake.wristSetPercentOuput(targetPowerCmd);
          }
      }
      else //Manual power is OFF
      {
          if(pidEnableCmd == false)//if we are still in manual mode and want to hold intake in place
          {
              targetPowerCmd = 0.0;
              intake.wristSetPercentOuput(targetPowerCmd);
          }
      }
   }

    if(pidEnableCmd)
    {
      intake.intakePIDLoopFunction();
    }


  
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    currentIntakeState = CatzStateUtil.IntakeState.FINISHED;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if(currentIntakeState == IntakeState.SET_STATE && intake.isIntakeInPos())
    {
      return true;
    }
    else
    {
      return false;
    }
  }
}

