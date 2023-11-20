// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils.CatzManipulatorPositions;
import frc.robot.Utils.CatzSharedDataUtil;
import frc.robot.Utils.CatzAbstractStateUtil;
import frc.robot.Utils.CatzAbstractStateUtil.GamePieceState;
import frc.robot.Utils.CatzAbstractStateUtil.SetMechanismState;
import frc.robot.CatzConstants.ManipulatorPoseConstants;
import frc.robot.subsystems.Arm.CatzArmSubsystem;
import frc.robot.subsystems.Elevator.CatzElevatorSubsystem;
import frc.robot.subsystems.Intake.CatzIntakeSubsystem;



public class ManipulatorToPoseCmd extends CommandBase {
  private static CatzElevatorSubsystem elevator = CatzElevatorSubsystem.getInstance();
  private static CatzIntakeSubsystem intake     = CatzIntakeSubsystem.getInstance();
  private static CatzArmSubsystem arm           = CatzArmSubsystem.getInstance();
  CatzManipulatorPositions targetPose;

  /** Creates a new SetStateCommand. */
  public ManipulatorToPoseCmd(CatzManipulatorPositions targetPose) {
    this.targetPose = targetPose;
    addRequirements(elevator, intake, arm);
  }


  public ManipulatorToPoseCmd(SetMechanismState currentMechansimState) {
    switch(currentMechansimState) {
      case SCORE_HIGH:
      if(CatzAbstractStateUtil.currentGamePieceState == GamePieceState.CONE) {
        targetPose = ManipulatorPoseConstants.SCORE_HIGH_CONE;
      }
      else{
        targetPose = ManipulatorPoseConstants.SCORE_HIGH_CUBE;
      }
      break;

      case SCORE_MID:
      if(CatzAbstractStateUtil.currentGamePieceState == GamePieceState.CONE) {
        targetPose = ManipulatorPoseConstants.SCORE_MID_CONE;
      }
      else{
        targetPose = ManipulatorPoseConstants.SCORE_MID_CUBE;
      }
      break;

      case SCORE_LOW:
      if(CatzAbstractStateUtil.currentGamePieceState == GamePieceState.CONE) {
        targetPose = ManipulatorPoseConstants.SCORE_LOW_CONE;
      }
      else{
        targetPose = ManipulatorPoseConstants.SCORE_LOW_CUBE;
      }
      break;

      case PICKUP_GROUND:
      if(CatzAbstractStateUtil.currentGamePieceState == GamePieceState.CONE) {
        targetPose = ManipulatorPoseConstants.PICKUP_CONE_GROUND;
      }
      else{
        targetPose = ManipulatorPoseConstants.PICKUP_CUBE_GROUND;
      }
      break;

      case PICKUP_SINGLE:
        targetPose = ManipulatorPoseConstants.PICKUP_CONE_SINGLE;
      break;

      default:
        targetPose = ManipulatorPoseConstants.STOW;
      break;
    }
    addRequirements(elevator, intake, arm);
  }

  @Override
  public void initialize() 
  {
    System.out.println("initstatemachine");
  }

  @Override
  public void execute() 
  {
    elevator.cmdUpdateElevator(targetPose);
    arm.cmdUpdateArm(targetPose);
    intake.cmdUpdateIntake(targetPose);

    Logger.getInstance().recordOutput("Commands/targetposeElevator", targetPose.getElevatorPosEnc());
    Logger.getInstance().recordOutput("Commands/targetposeArm", targetPose.getArmPosEnc());
    Logger.getInstance().recordOutput("Commands/targetposeIntake", targetPose.getWristAngleDeg());
  }

  @Override
  public void end(boolean interrupted) {
      
  }

  @Override
  public boolean isFinished() {
      return (!CatzSharedDataUtil.sharedElevatorInPos && 
              !CatzSharedDataUtil.sharedIntakeInPos && 
              !CatzSharedDataUtil.sharedArmInPos);
  }
}
