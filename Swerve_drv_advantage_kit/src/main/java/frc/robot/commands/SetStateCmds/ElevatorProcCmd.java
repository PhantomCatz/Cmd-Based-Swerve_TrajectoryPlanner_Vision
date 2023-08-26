// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SetStateCmds;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.CatzConstants;
import frc.robot.Utils.CatzStateUtil;
import frc.robot.Utils.CatzStateUtil.GamePieceState;
import frc.robot.subsystems.Arm.CatzArmSubsystem;
import frc.robot.subsystems.Elevator.CatzElevatorSubsystem;

public class ElevatorProcCmd extends CommandBase {
  CatzElevatorSubsystem elevator = CatzElevatorSubsystem.getInstance();
  CatzArmSubsystem arm = CatzArmSubsystem.getInstance();
  CatzStateUtil.SetMechanismState currentMechanismState;

  private final double ARM_ENCODER_THRESHOLD = 35000.0;

  private boolean elevatorDescent;
  private double targetPosition = -999.0;
  private double currentPosition = -999.0;
  private double positionError = -999.0;

  private final double ELEVATOR_POS_ERROR_THRESHOLD = 1000.0; //0.424 inches

  private final double NO_TARGET_POSITION = -999999.0;

  private boolean elevatorInPosition = false;

  private int numConsectSamples = 0;
  /** Creates a new ElevatorCmd. */
  public ElevatorProcCmd(CatzStateUtil.SetMechanismState currentMechState) 
  {
    this.currentMechanismState = currentMechState;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    elevatorDescent = false;
    elevatorInPosition = false;

      switch(currentMechanismState)
      {
          case STOW:
          case PICKUP_GROUND :
          case SCORE_LOW :

              elevatorDescent = true;
              targetPosition = CatzConstants.ELEVATOR_POS_ENC_CNTS_LOW;
              break;

          case PICKUP_SINGLE :
            if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
              {
                elevatorDescent = true;
                targetPosition = CatzConstants.ELEVATOR_POS_ENC_CNTS_LOW;
              }
            else
              {
                elevator.elevatorSetToSinglePickup();
                targetPosition = CatzConstants.ELEVATOR_POS_ENC_CNTS_HIGH;
              }
              break;
          case SCORE_MID :
            if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
              {
                elevator.elevatorSetToMidPosCube();
                targetPosition = CatzConstants.ELEVATOR_POS_ENC_CNTS_MID_CUBE;
              }
            else
              {
                elevator.elevatorSetToMidPosCone();
                targetPosition = CatzConstants.ELEVATOR_POS_ENC_CNTS_MID_CONE;
              }
              break;
              
          case SCORE_HIGH :
                elevator.elevatorSetToHighPos();
                targetPosition = CatzConstants.ELEVATOR_POS_ENC_CNTS_HIGH;
              break;

          default:
              //TBD
              break;
      }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    if((elevatorDescent == true) && (arm.getArmEncoder() <= ARM_ENCODER_THRESHOLD))
    {
      elevator.elevatorSetToLowPos();
    }


    currentPosition = elevator.getElevatorEncoder();
    positionError = currentPosition - targetPosition;
    if  ((Math.abs(positionError) <= ELEVATOR_POS_ERROR_THRESHOLD) && targetPosition != NO_TARGET_POSITION) 
    {

        targetPosition = NO_TARGET_POSITION;
        numConsectSamples++;
            if(numConsectSamples >= 10) {   
                elevatorInPosition = true;
            }
    }
    else 
    {
        numConsectSamples = 0;
    }

    Logger.getInstance().recordOutput("/Elevator/Elevator Position", elevator.getElevatorEncoder());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if(elevatorInPosition == true)
    {    
      return true;
    }
    else
    {
      return false;
    }
  }
}
