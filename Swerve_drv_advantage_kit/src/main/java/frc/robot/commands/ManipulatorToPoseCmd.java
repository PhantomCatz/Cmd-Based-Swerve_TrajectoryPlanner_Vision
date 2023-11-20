// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils.CatzManipulatorPositions;
import frc.robot.Utils.CatzSharedDataUtil;
import frc.robot.Utils.CatzAbstractStateUtil;
import frc.robot.Utils.CatzAbstractStateUtil.GamePieceState;
import frc.robot.Utils.CatzAbstractStateUtil.SetMechanismState;
import frc.robot.RobotContainer;
import frc.robot.CatzConstants.ManipulatorPoseConstants;
import frc.robot.RobotContainer.mechMode;
import frc.robot.subsystems.Arm.CatzArmSubsystem;
import frc.robot.subsystems.Elevator.CatzElevatorSubsystem;
import frc.robot.subsystems.Intake.CatzIntakeSubsystem;



public class ManipulatorToPoseCmd extends CommandBase {
  private static CatzElevatorSubsystem m_elevator = CatzElevatorSubsystem.getInstance();
  private static CatzIntakeSubsystem m_intake     = CatzIntakeSubsystem.getInstance();
  private static CatzArmSubsystem m_arm           = CatzArmSubsystem.getInstance();
  CatzManipulatorPositions m_targetPose;

  /** Creates a new SetStateCommand. */
  public ManipulatorToPoseCmd(CatzManipulatorPositions targetPose) {
    this.m_targetPose = targetPose;
    addRequirements(m_elevator, m_intake, m_arm);
  }


  public ManipulatorToPoseCmd(SetMechanismState currentMechansimState) {
    switch(currentMechansimState) {
      case SCORE_HIGH:
      if(CatzAbstractStateUtil.currentGamePieceState == GamePieceState.CONE) {
        m_targetPose = ManipulatorPoseConstants.SCORE_HIGH_CONE;
      }
      else{
        m_targetPose = ManipulatorPoseConstants.SCORE_HIGH_CUBE;
      }
      break;

      case SCORE_MID:
      if(CatzAbstractStateUtil.currentGamePieceState == GamePieceState.CONE) {
        m_targetPose = ManipulatorPoseConstants.SCORE_MID_CONE;
      }
      else{
        m_targetPose = ManipulatorPoseConstants.SCORE_MID_CUBE;
      }
      break;

      case SCORE_LOW:
      if(CatzAbstractStateUtil.currentGamePieceState == GamePieceState.CONE) {
        m_targetPose = ManipulatorPoseConstants.SCORE_LOW_CONE;
      }
      else{
        m_targetPose = ManipulatorPoseConstants.SCORE_LOW_CUBE;
      }
      break;

      case PICKUP_GROUND:
      if(CatzAbstractStateUtil.currentGamePieceState == GamePieceState.CONE) {
        m_targetPose = ManipulatorPoseConstants.PICKUP_CONE_GROUND;
      }
      else{
        m_targetPose = ManipulatorPoseConstants.PICKUP_CUBE_GROUND;
      }
      break;

      case PICKUP_SINGLE:
        m_targetPose = ManipulatorPoseConstants.PICKUP_CONE_SINGLE;
      break;

      default:
        m_targetPose = ManipulatorPoseConstants.STOW;
      break;
    }
    addRequirements(m_elevator, m_intake, m_arm);
  }

  @Override
  public void initialize() 
  {
    System.out.println("initstatemachine");
    RobotContainer.armControlMode = mechMode.AutoMode;
    RobotContainer.elevatorControlMode = mechMode.AutoMode;
    RobotContainer.intakeControlMode = mechMode.AutoMode;
  }

  @Override
  public void execute() 
  {
    m_elevator.cmdUpdateElevator(m_targetPose);
    m_arm.cmdUpdateArm(m_targetPose);
    m_intake.cmdUpdateIntake(m_targetPose);

    Logger.getInstance().recordOutput("Commands/targetposeElevator", m_targetPose.getElevatorPosEnc());
    Logger.getInstance().recordOutput("Commands/targetposeArm", m_targetPose.getArmPosEnc());
    Logger.getInstance().recordOutput("Commands/targetposeIntake", m_targetPose.getWristAngleDeg());
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
