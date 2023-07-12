// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MechanismCmds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.CatzConstants;
import frc.robot.Utils.CatzStateUtil;
import frc.robot.subsystems.Arm.CatzArmSubsystem;
import frc.robot.subsystems.Elevator.CatzElevatorSubsystem;
import frc.robot.subsystems.Intake.CatzIntakeSubsytem;

public class MechanismCommand extends CommandBase 
{
  CatzElevatorSubsystem elevator;
  CatzArmSubsystem arm;
  CatzIntakeSubsytem intake;
  CatzStateUtil.MechanismState currentMechState;
  CatzStateUtil.GamePieceState currentGamePieceState;
  public MechanismCommand(CatzElevatorSubsystem elevator, 
                          CatzArmSubsystem arm, 
                          CatzIntakeSubsytem intake, 
                          CatzStateUtil.MechanismState currentMechState) 
  {
    this.currentMechState = currentMechState;


    addRequirements(elevator, arm, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    
    switch(currentMechState)
    {
      case ScoreHigh:
        switch(currentGamePieceState)
        {
          case CONE:

          break;
          case CUBE:

          break;
          default:

          break;
        }
      break;
      case ScoreMid:
        switch(currentGamePieceState)
        {
          case CONE:
          
          break;
          case CUBE:

          break;
          default:

          break;
        }
      break;
      case ScoreLow:
        switch(currentGamePieceState)
        {
          case CONE:
          
          break;
          case CUBE:

          break;
          default:

          break;
        }
      break;
      case PickupLow:

      break;
      case PickupSingle:

      break;
      case PickupDouble:

      break;

      default:
        break;
    }

  }

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
