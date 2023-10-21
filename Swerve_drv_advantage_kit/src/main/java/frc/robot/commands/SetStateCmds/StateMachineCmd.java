// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SetStateCmds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
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


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StateMachineCmd extends InstantCommand {
  CatzElevatorSubsystem elevator = CatzElevatorSubsystem.getInstance();
  CatzIntakeSubsystem intake     = CatzIntakeSubsystem.getInstance();
  CatzArmSubsystem arm           = CatzArmSubsystem.getInstance();
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
        arm.cmdProcArm(ArmAutoState.RETRACT);
        intake.cmdProcIntake(CatzConstants.IntakeConstants.STOW_ENC_POS);
        elevator.cmdProcElevator(ElevatorAutoState.LOW);
        break;
            
        case PICKUP_GROUND :
        arm.cmdProcArm(ArmAutoState.PICKUP);
        elevator.cmdProcElevator(ElevatorAutoState.LOW);
          if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
            {
              intake.cmdProcIntake(CatzConstants.IntakeConstants.INTAKE_CUBE_ENC_POS);
            }
          else
            {
              intake.cmdProcIntake(CatzConstants.IntakeConstants.INTAKE_CONE_ENC_POS_GROUND);
            }
        break;

        case PICKUP_SINGLE :
          if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
            {

            }
          else
            {
              arm.cmdProcArm(ArmAutoState.PICKUP);
              elevator.cmdProcElevator(ElevatorAutoState.LOW);
              intake.cmdProcIntake(IntakeConstants.INTAKE_CONE_ENC_POS_SINGLE_UPRIGHT);
            }
        break;
            
        case SCORE_LOW :
        arm.cmdProcArm(ArmAutoState.PICKUP);
        elevator.cmdProcElevator(ElevatorAutoState.LOW);
          if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
            {
              intake.cmdProcIntake(IntakeConstants.SCORE_CUBE_ENC_POS);
            }
          else
            {
              intake.cmdProcIntake(IntakeConstants.SCORE_CONE_LOW_ENC_POS);
            }
        break;

        case SCORE_MID :
        arm.cmdProcArm(ArmAutoState.RETRACT);
        elevator.cmdProcElevator(ElevatorAutoState.MIDCUBE);
          if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
            {
              intake.cmdProcIntake(IntakeConstants.SCORE_CUBE_ENC_POS);
            }
          else
            {
              intake.cmdProcIntake(IntakeConstants.SCORE_CONE_MID_ENC_POS);
            }
        break;
            
        case SCORE_HIGH :
        arm.cmdProcArm(ArmAutoState.EXTEND);
        elevator.cmdProcElevator(ElevatorAutoState.HIGH);
          if(CatzStateUtil.currentGamePieceState == GamePieceState.CUBE)
            {
              intake.cmdProcIntake(IntakeConstants.SCORE_CUBE_ENC_POS);
            }
          else
            {
              intake.cmdProcIntake(IntakeConstants.SCORE_CONE_HIGH_ENC_POS_AUTON);
            }
        break;

        default:
            //TBD
            break;
    }
  }
  
}
