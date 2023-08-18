// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MechanismCmds;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Utils.CatzStateUtil;
import frc.robot.Utils.CatzStateUtil.ArmState;
import frc.robot.Utils.CatzStateUtil.ElevatorState;
import frc.robot.Utils.CatzStateUtil.IntakeState;
import frc.robot.subsystems.Elevator.CatzElevatorSubsystem;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetStateCmdGroup extends ParallelCommandGroup {
  CatzElevatorSubsystem elevator = CatzElevatorSubsystem.getInstance();
  /** Creates a new SetStateCommand. */
  public SetStateCmdGroup(CatzStateUtil.SetMechanismState currentMechanismState) 
  {
    addCommands(new ElevatorProcCmd(ElevatorState.SET_STATE, currentMechanismState, null, null));
    addCommands(new ArmProcCmd(ArmState.SET_STATE, currentMechanismState, false, false));
    addCommands(new IntakeProcCmd(IntakeState.SET_STATE, currentMechanismState, null, null));
  }
}
