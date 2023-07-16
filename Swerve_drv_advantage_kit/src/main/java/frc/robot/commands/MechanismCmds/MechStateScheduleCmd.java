// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MechanismCmds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.CatzConstants;
import frc.robot.Utils.CatzMathUtils;
import frc.robot.Utils.CatzStateUtil;
import frc.robot.subsystems.Arm.CatzArmSubsystem;
import frc.robot.subsystems.Elevator.CatzElevatorSubsystem;
import frc.robot.subsystems.Intake.CatzIntakeSubsystem;

public class MechStateScheduleCmd extends InstantCommand 
{
  CatzElevatorSubsystem elevator;
  CatzArmSubsystem arm;
  CatzIntakeSubsystem intake;
  CatzStateUtil.MechanismState currentMechState;
  public MechStateScheduleCmd(CatzElevatorSubsystem elevator, 
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
    new ElevatorCmd(elevator, currentMechState, CatzStateUtil.ElevatorState.SET_STATE, null, null);
    new ArmCmd(arm, currentMechState, CatzStateUtil.ArmState.SET_STATE, false, false);
    new IntakeCmd(intake, currentMechState, CatzStateUtil.IntakeState.SET_STATE, null, null);
  }
}
