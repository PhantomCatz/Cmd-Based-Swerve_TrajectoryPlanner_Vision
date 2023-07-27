// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MechanismCmds;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.CatzConstants;
import frc.robot.Utils.CatzStateUtil;
import frc.robot.Utils.CatzStateUtil.ElevatorState;
import frc.robot.Utils.CatzStateUtil.GamePieceState;
import frc.robot.subsystems.Arm.CatzArmSubsystem;
import frc.robot.subsystems.Elevator.CatzElevatorSubsystem;

public class ElevatorCmd extends CommandBase {
  CatzElevatorSubsystem elevator = CatzElevatorSubsystem.getInstance();
  CatzArmSubsystem arm = CatzArmSubsystem.getInstance();
  CatzStateUtil.MechanismState currentMechanismState;
  CatzStateUtil.ElevatorState currentElevatorState;
  Supplier<Double> supplierElevatorPwr;
  Supplier<Boolean> supplierManualMode;

  private final double MANUAL_CONTROL_DEADBAND = 0.1;
  private final double MANUAL_HOLD_STEP_SIZE = 10000.0; //5000.0;
  private final double ARM_ENCODER_THRESHOLD = 35000.0;

  private double targetPositionEnc;
  private boolean elevatorDescent;
  private double targetPosition = -999.0;
  private double currentPosition = -999.0;
  private double positionError = -999.0;

  private final double ELEVATOR_POS_ERROR_THRESHOLD = 1000.0; //0.424 inches

  private final double NO_TARGET_POSITION = -999999.0;

  private boolean elevatorInPosition = false;

  private int numConsectSamples = 0;
  /** Creates a new ElevatorCmd. */
  public ElevatorCmd(CatzStateUtil.ElevatorState currentElevatorState, 
                     CatzStateUtil.MechanismState currentMechState,
                     Supplier<Double> supplierElevatorPwr, Supplier<Boolean> supplierManualMode) 
  {
    this.currentElevatorState = currentElevatorState;
    this.currentMechanismState = currentMechState;
    this.supplierElevatorPwr = supplierElevatorPwr;
    this.supplierManualMode = supplierManualMode;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    elevatorDescent = false;

    if(currentElevatorState == ElevatorState.SET_STATE)
    {
      switch(currentMechanismState)
      {
          case STOW:
          case PICKUP_GROUND :
          case SCORE_LOW :

              elevatorDescent = true;
              targetPosition = CatzConstants.POS_ENC_CNTS_LOW;
              break;

          case PICKUP_SINGLE :
            if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
              {
                elevatorDescent = true;
                targetPosition = CatzConstants.POS_ENC_CNTS_LOW;
              }
            else
              {
                elevator.elevatorSetToSinglePickup();
                targetPosition = CatzConstants.POS_ENC_CNTS_HIGH;
                
              }
              break;
          case SCORE_MID :
            if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
              {
                elevator.elevatorSetToMidPosCube();
                targetPosition = CatzConstants.POS_ENC_CNTS_MID_CUBE;
              }
            else
              {
                elevator.elevatorSetToMidPosCone();
                targetPosition = CatzConstants.POS_ENC_CNTS_MID_CONE;
              }
              break;
              
          case SCORE_HIGH :
                elevator.elevatorSetToHighPos();
                targetPosition = CatzConstants.POS_ENC_CNTS_HIGH;
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
    boolean isElevatorInManualMode = supplierManualMode.get();
    double elevatorPwr = supplierElevatorPwr.get();
    
    if(currentElevatorState == ElevatorState.MANUAL)
    {
      
      if(Math.abs(elevatorPwr) >= MANUAL_CONTROL_DEADBAND)
      {
          if(isElevatorInManualMode) // Full manual
          {
              elevator.elevatorManual(elevatorPwr);
          }
          else // Hold Position
          {
              targetPositionEnc = elevator.getElevatorEncoder();
              targetPositionEnc = targetPositionEnc + (elevatorPwr * MANUAL_HOLD_STEP_SIZE);
              elevator.elevatorHoldingManual(targetPositionEnc);
          }
      }
      else
      {
          if (isElevatorInManualMode)
          {
              elevator.elevatorManual(0.0);
          }
      }
    }


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

    Logger.getInstance().recordOutput("/Elevator/Elevator Position", currentElevatorState.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    currentElevatorState = CatzStateUtil.ElevatorState.FINISHED;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if(currentElevatorState == ElevatorState.SET_STATE && elevatorInPosition == true)
    {    
      return true;
    }
    else
    {
      return false;
    }
  }
}
