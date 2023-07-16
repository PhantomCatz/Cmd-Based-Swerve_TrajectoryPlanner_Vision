// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MechanismCmds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils.CatzStateUtil;
import frc.robot.subsystems.Elevator.CatzElevatorSubsystem;

public class ElevatorCmd extends CommandBase {
  CatzElevatorSubsystem elevator;
  /** Creates a new ElevatorCmd. */
  public ElevatorCmd(CatzElevatorSubsystem elevator, 
                     CatzStateUtil.MechanismState currentMechState, 
                     CatzStateUtil.ElevatorState currentElevatorState, Supplier<Double> elevatorPwr, Supplier<Boolean> manualMode) {
    this.elevator = elevator;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
