// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Utils.CatzStateUtil;
import frc.robot.commands.MechanismCmds.ArmCmd;
import frc.robot.commands.MechanismCmds.ElevatorCmd;
import frc.robot.commands.MechanismCmds.IntakeCmd;
import frc.robot.subsystems.Arm.CatzArmSubsystem;
import frc.robot.subsystems.Elevator.CatzElevatorSubsystem;
import frc.robot.subsystems.Intake.CatzIntakeSubsystem;

public class SetStateCmd extends InstantCommand 
{
  CatzElevatorSubsystem elevator;
  CatzArmSubsystem arm;
  CatzIntakeSubsystem intake;
  CatzStateUtil.MechanismState currentMechState;
  public SetStateCmd(CatzElevatorSubsystem elevator, 
                              CatzArmSubsystem arm, 
                              CatzIntakeSubsystem intake, 
                              CatzStateUtil.MechanismState currentMechState) 
  {
    this.currentMechState = currentMechState;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    new ElevatorCmd(elevator, CatzStateUtil.ElevatorState.SET_STATE, currentMechState, null, null);
    new ArmCmd(arm, CatzStateUtil.ArmState.SET_STATE, currentMechState, false, false);
    new IntakeCmd(intake, CatzStateUtil.IntakeState.SET_STATE, currentMechState, null, null);
  }
}
