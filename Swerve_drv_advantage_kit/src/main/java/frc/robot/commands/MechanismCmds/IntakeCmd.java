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

  private double   targetPower = 0.0;
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
              intake.targetPositionDeg = intake.STOW_ENC_POS;
              break;
              
          case PICKUPGROUND :
            if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
              {
                intake.targetPositionDeg = intake.INTAKE_CUBE_ENC_POS;
              }
            else
              {
                intake.targetPositionDeg = intake.INTAKE_PICKUP_CONE_ENC_POS_GROUND;
              }
              break;

          case PickupSingle :
            if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
              {

              }
            else
              {
                intake.targetPositionDeg = intake.INTAKE_PICKUP_CONE_ENC_POS_SINGLE;
              }
              break;
              
          case SCORELOW :
            if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
              {
                intake.targetPositionDeg = intake.SCORE_CUBE_ENC_POS;
              }
            else
              {
                intake.targetPositionDeg = intake.SCORE_CONE_LOW_ENC_POS;
              }
              break;

          case ScoreMid :
            if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
              {
                  intake.targetPositionDeg = intake.SCORE_CUBE_ENC_POS;
              }
            else
              {
                intake.targetPositionDeg = intake.SCORE_CONE_MID_ENC_POS;
              }
              break;
              
          case ScoreHigh :
            if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
              {
                intake.targetPositionDeg = intake.SCORE_CUBE_ENC_POS;
              }
            else
              {
                intake.targetPositionDeg = intake.SCORE_CONE_HIGH_ENC_POS; 
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
                intake.targetPositionDeg = Math.min((intake.targetPositionDeg + wristPwr * MANUAL_HOLD_STEP_SIZE), SOFT_LIMIT_FORWARD);
            }
            else
            {
              intake.targetPositionDeg = Math.max((intake.targetPositionDeg + wristPwr * MANUAL_HOLD_STEP_SIZE), SOFT_LIMIT_REVERSE);
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
      intake.intakePIDControl();
    }
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

