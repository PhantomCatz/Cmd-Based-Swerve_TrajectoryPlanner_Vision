// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils.CatzStateUtil;
import frc.robot.Utils.CatzStateUtil.ArmState;
import frc.robot.Utils.CatzStateUtil.ElevatorState;
import frc.robot.Utils.CatzStateUtil.IntakeState;
import frc.robot.commands.MechanismCmds.ArmCmd;
import frc.robot.commands.MechanismCmds.ElevatorCmd;
import frc.robot.commands.MechanismCmds.IntakeCmd;

public class SetStateCmd extends CommandBase 
{
  CatzStateUtil.MechanismState currentMechState;


  private Timer stateTimer;

  public SetStateCmd(CatzStateUtil.MechanismState currentMechState) 
  {
    this.currentMechState = currentMechState;
    stateTimer = new Timer();
  }

  // Called when the command is initially scheduled...distributes the currentMechstate enum to all the mechanisms...making calls to the stateMachine easier
  @Override
  public void initialize() 
  {
    stateTimer.reset();
    stateTimer.start();
    new ElevatorCmd(CatzStateUtil.ElevatorState.SET_STATE, 
                    currentMechState,
            null,
             null);

    new ArmCmd(CatzStateUtil.ArmState.SET_STATE,
               currentMechState,
               false,
              false);

    new IntakeCmd(CatzStateUtil.IntakeState.SET_STATE, 
                  currentMechState,
                    null,
          null);

    Timer.delay(1);
  }

  @Override
  public void execute()
  {
  }

  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    if(CatzStateUtil.currentElevatorState == ElevatorState.FINISHED &&
       CatzStateUtil.currentArmState == ArmState.FINISHED &&
       CatzStateUtil.currentIntakeState == IntakeState.FINISHED)
       {
          return true;
       }
    else if(stateTimer.get() >= 2)
       {
         return true;
       }
    else
       {
          return false;
       }
  }
}
