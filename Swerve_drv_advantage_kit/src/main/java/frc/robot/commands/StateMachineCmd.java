// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  private SetMechanismState currentMechanismState;

  /** Creates a new SetStateCommand. */
  public StateMachineCmd(CatzStateUtil.SetMechanismState currentMechanismState) 
  {
    this.currentMechanismState = currentMechanismState;
    addRequirements(elevator, intake, arm);
  }
  @Override
  public void initialize() 
  {
    switch(currentMechanismState)
    {
        case  STOW:
        arm.cmdUpdateArm(ArmAutoState.RETRACT);
        intake.cmdUpdateIntake(CatzConstants.IntakeConstants.STOW_ENC_POS);
        elevator.cmdUpdateElevator(ElevatorAutoState.LOW);
        break;
            
        case PICKUP_GROUND :
        arm.cmdUpdateArm(ArmAutoState.PICKUP);
        elevator.cmdUpdateElevator(ElevatorAutoState.LOW);
          if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
            {
              intake.cmdUpdateIntake(CatzConstants.IntakeConstants.INTAKE_CUBE_ENC_POS);
            }
          else
            {
              intake.cmdUpdateIntake(CatzConstants.IntakeConstants.INTAKE_CONE_ENC_POS_GROUND);
            }
        break;

        case PICKUP_SINGLE :
          if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
            {

            }
          else
            {
              arm.cmdUpdateArm(ArmAutoState.PICKUP);
              elevator.cmdUpdateElevator(ElevatorAutoState.LOW);
              intake.cmdUpdateIntake(IntakeConstants.INTAKE_CONE_ENC_POS_SINGLE_UPRIGHT);
            }
        break;
            
        case SCORE_LOW :
        arm.cmdUpdateArm(ArmAutoState.PICKUP);
        elevator.cmdUpdateElevator(ElevatorAutoState.LOW);
          if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
            {
              intake.cmdUpdateIntake(IntakeConstants.SCORE_CUBE_ENC_POS);
            }
          else
            {
              intake.cmdUpdateIntake(IntakeConstants.SCORE_CONE_LOW_ENC_POS);
            }
        break;

        case SCORE_MID :
        arm.cmdUpdateArm(ArmAutoState.RETRACT);
        elevator.cmdUpdateElevator(ElevatorAutoState.MIDCUBE);
          if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
            {
              intake.cmdUpdateIntake(IntakeConstants.SCORE_CUBE_ENC_POS);
            }
          else
            {
              intake.cmdUpdateIntake(IntakeConstants.SCORE_CONE_MID_ENC_POS);
            }
        break;
            
        case SCORE_HIGH :
        arm.cmdUpdateArm(ArmAutoState.EXTEND);
        elevator.cmdUpdateElevator(ElevatorAutoState.HIGH);
          if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
            {
              intake.cmdUpdateIntake(IntakeConstants.SCORE_CUBE_ENC_POS);
            }
          else
            {
              intake.cmdUpdateIntake(IntakeConstants.SCORE_CONE_HIGH_ENC_POS_AUTON);
            }
        break;

        default:
            //TBD
            break;
    }
  }

  @Override
  public void execute() {
      
  }

  @Override
  public void end(boolean interrupted) {
      
  }

  @Override
  public boolean isFinished() {
      return false;
  }
  
}
