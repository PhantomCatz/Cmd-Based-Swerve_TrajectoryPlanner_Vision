// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MechanismCmds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils.CatzStateUtil;
import frc.robot.Utils.CatzStateUtil.ElevatorState;
import frc.robot.Utils.CatzStateUtil.GamePieceState;
import frc.robot.subsystems.Elevator.CatzElevatorSubsystem;

public class ElevatorCmd extends CommandBase {
  CatzElevatorSubsystem elevator;
  CatzStateUtil.MechanismState currentMechanismState;
  CatzStateUtil.ElevatorState currentElevatorState;
  Supplier<Double> supplierElevatorPwr;
  Supplier<Boolean> supplierManualMode;

  private final double MANUAL_CONTROL_DEADBAND = 0.1;
  private final double MANUAL_HOLD_STEP_SIZE = 10000.0; //5000.0;

  private boolean elevatorInManual;
  private double targetPositionEnc;
  /** Creates a new ElevatorCmd. */
  public ElevatorCmd(CatzElevatorSubsystem elevator, 
                     CatzStateUtil.ElevatorState currentElevatorState, 
                     CatzStateUtil.MechanismState currentMechState,
                     Supplier<Double> supplierElevatorPwr, Supplier<Boolean> supplierManualMode) 
  {
    this.elevator = elevator;
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
    if(currentElevatorState == ElevatorState.SET_STATE)
    {
      switch(currentMechanismState)
      {
          case STOW:
          case PICKUPGROUND :
          case SCORELOW :
              elevator.elevatorSetToLowPos();
              break;

          case PickupSingle :
            if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
              {

              }
            else
              {
                elevator.elevatorSetToSinglePickup();
              }
              break;
          case ScoreMid :
            if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
              {
                elevator.elevatorSetToMidPosCube();
              }
            else
              {
                elevator.elevatorSetToMidPosCone();
              }
              break;
              
          case ScoreHigh :
                elevator.elevatorSetToHighPos();
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
    double elevatorPwr = supplierElevatorPwr.get();
    
    if(currentElevatorState == ElevatorState.MANUAL)
    {
      if(manualMode)
      {
          elevatorInManual = true;
      }
      
      if(Math.abs(elevatorPwr) >= MANUAL_CONTROL_DEADBAND)
      {
          if(elevatorInManual) // Full manual
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
          if (elevatorInManual)
          {
              elevator.elevatorManual(0.0);
          }
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
