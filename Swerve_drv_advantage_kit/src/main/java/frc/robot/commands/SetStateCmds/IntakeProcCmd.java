// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SetStateCmds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.CatzConstants;
import frc.robot.Utils.CatzStateUtil;
import frc.robot.Utils.CatzStateUtil.GamePieceState;
import frc.robot.subsystems.Intake.CatzIntakeSubsystem;

public class IntakeProcCmd extends CommandBase {
  CatzIntakeSubsystem intake = CatzIntakeSubsystem.getInstance();
  CatzStateUtil.SetMechanismState currentMechanismState;
  Supplier<Double> supplierWristPwr;
  Supplier<Boolean> supplierManualMode;


  private double   targetPosDegCmd;



  private int numConsectSamples = 0;
  /** Creates a new IntakeCmd. */
  public IntakeProcCmd(CatzStateUtil.SetMechanismState currentMechState) {

    this.currentMechanismState = currentMechState;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

      intake.setPIDEnable(true);

      switch(currentMechanismState)
      {
          case  STOW:
            targetPosDegCmd = CatzConstants.IntakeConstants.STOW_ENC_POS;
              break;
              
          case PICKUP_GROUND :
            if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
              {
                targetPosDegCmd = CatzConstants.IntakeConstants.INTAKE_CUBE_ENC_POS;
              }
            else
              {
                targetPosDegCmd = CatzConstants.IntakeConstants.INTAKE_PICKUP_CONE_ENC_POS_GROUND;
              }
              break;

          case PICKUP_SINGLE :
            if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
              {

              }
            else
              {
                targetPosDegCmd = CatzConstants.IntakeConstants.INTAKE_PICKUP_CONE_ENC_POS_SINGLE;
              }
              break;
              
          case SCORE_LOW :
            if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
              {
                targetPosDegCmd = CatzConstants.IntakeConstants.SCORE_CUBE_ENC_POS;
              }
            else
              {
                targetPosDegCmd = CatzConstants.IntakeConstants.SCORE_CONE_LOW_ENC_POS;
              }
              break;

          case SCORE_MID :
            if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
              {
                targetPosDegCmd = CatzConstants.IntakeConstants.SCORE_CUBE_ENC_POS;
              }
            else
              {
                targetPosDegCmd = CatzConstants.IntakeConstants.SCORE_CONE_MID_ENC_POS;
              }
              break;
              
          case SCORE_HIGH :
            if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
              {
                targetPosDegCmd = CatzConstants.IntakeConstants.SCORE_CUBE_ENC_POS;
              }
            else
              {
                targetPosDegCmd = CatzConstants.IntakeConstants.SCORE_CONE_HIGH_ENC_POS; 
              }
              break;

          default:
              //TBD
              break;
      }
      intake.setTargetPositionDeg(targetPosDegCmd);
}


@Override
public void execute() 
{

 intake.IntakePIDLoop();

}

@Override
public void end(boolean interrupted) 
{
    
}
@Override
public boolean isFinished() 
{
  if(intake.isIntakeInPos())
  {
    return true;
  }
  else
  {
    return false;
  }
}

}

