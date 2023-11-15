// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Utils.CatzManipulatorPositions;
import frc.robot.Utils.CatzStateUtil;
import frc.robot.Utils.CatzStateUtil.GamePieceState;
import frc.robot.Utils.CatzStateUtil.SetMechanismState;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.IntakeConstants;
import frc.robot.subsystems.Arm.CatzArmSubsystem;
import frc.robot.subsystems.Arm.CatzArmSubsystem.ArmAutoState;
import frc.robot.subsystems.Elevator.CatzElevatorSubsystem;
import frc.robot.subsystems.Elevator.CatzElevatorSubsystem.ElevatorAutoState;
import frc.robot.subsystems.Intake.CatzIntakeSubsystem;



public class StateMachineCmd extends CommandBase {
  private static CatzElevatorSubsystem elevator = CatzElevatorSubsystem.getInstance();
  private static CatzIntakeSubsystem intake     = CatzIntakeSubsystem.getInstance();
  private static CatzArmSubsystem arm           = CatzArmSubsystem.getInstance();
  Supplier<CatzManipulatorPositions> targetPoseSupplier;
  CatzManipulatorPositions targetPose;
  private SetMechanismState currentMechanismState;

  /** Creates a new SetStateCommand. */
  public StateMachineCmd(Supplier<CatzManipulatorPositions> targetPoseSupplier) 
  {
    this.targetPoseSupplier = targetPoseSupplier;
    addRequirements(elevator, intake, arm);
  }
  public StateMachineCmd(CatzManipulatorPositions targetPose) 
  {
    this(()-> targetPose);
  }
  @Override
  public void initialize() 
  {

  }

  @Override
  public void execute() 
  {
    CatzManipulatorPositions targetPose;
    targetPose =  targetPoseSupplier.get();
    elevator.cmdUpdateElevator(targetPose);
    arm.cmdUpdateArm(targetPose);
    intake.cmdUpdateIntake(targetPose);
  }

  @Override
  public void end(boolean interrupted) {
      
  }

  @Override
  public boolean isFinished() {
      return false;
  }
  
}
