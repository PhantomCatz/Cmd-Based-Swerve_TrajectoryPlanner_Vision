// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Utils.CatzStateUtil;
import frc.robot.Utils.CatzStateUtil.ArmState;
import frc.robot.Utils.CatzStateUtil.ElevatorState;
import frc.robot.Utils.CatzStateUtil.IntakeState;
import frc.robot.commands.MechanismCmds.ArmCmd;
import frc.robot.commands.MechanismCmds.ElevatorCmd;
import frc.robot.commands.MechanismCmds.IntakeCmd;

public class SetStateCmd extends InstantCommand 
{
  CatzStateUtil.MechanismState currentMechState;



  public SetStateCmd(CatzStateUtil.MechanismState currentMechState) 
  {
    this.currentMechState = currentMechState;

  }

  // Called when the command is initially scheduled...distributes the currentMechstate enum to all the mechanisms...making calls to the stateMachine easier
  @Override
  public void initialize() 
  {
    new ElevatorCmd(CatzStateUtil.ElevatorState.SET_STATE, 
                    currentMechState,
             () -> null,
             () -> null);

    new ArmCmd(CatzStateUtil.ArmState.SET_STATE,
               currentMechState,
               false,
              false);
              
/* 
    new IntakeCmd(CatzStateUtil.IntakeState.SET_STATE, 
                  currentMechState,
                    null,
          null);
          */
    System.out.println("cmds");
  }

  @Override
  public void execute()
  {
  }

  // Returns true when the command should end.
}
