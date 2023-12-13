// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualStateCmds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.mechMode;
import frc.robot.Utils.CatzManipulatorPositions;
import frc.robot.Utils.CatzSharedDataUtil;
import frc.robot.subsystems.Elevator.SubsystemCatzElevator;

public class ElevatorManualCmd extends CommandBase {
  SubsystemCatzElevator m_elevator = SubsystemCatzElevator.getInstance();
      
  Supplier<Double> supplierElevatorPwr;
  Supplier<Boolean> supplierManualMode;

  private final double MANUAL_CONTROL_DEADBAND = 0.1;
  private final double MANUAL_HOLD_STEP_SIZE = 10000.0; //5000.0;

  private double manualHoldTargetPos;


  /** Creates a new ManualElevatorCmd. */
  public ElevatorManualCmd(Supplier<Double> supplierElevatorPwr, 
                           Supplier<Boolean> supplierManualMode) {
    this.supplierElevatorPwr = supplierElevatorPwr;
    this.supplierManualMode = supplierManualMode;
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.elevatorControlMode = mechMode.AutoMode;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    boolean isElevatorInManualMode = supplierManualMode.get();
    double  elevatorPwr = -supplierElevatorPwr.get(); //reverse elevator pwr so up = up on joystick
    CatzManipulatorPositions targetPosition;
    
      if(Math.abs(elevatorPwr) >= MANUAL_CONTROL_DEADBAND)
      {//if pwr is set
          if(isElevatorInManualMode) // Full manual
          {
              m_elevator.elevatorFullManualCmd(elevatorPwr);
          }
          else // Hold Position
          {
            manualHoldTargetPos = CatzSharedDataUtil.sharedElevatorEncCnts;
            manualHoldTargetPos = manualHoldTargetPos + (elevatorPwr * MANUAL_HOLD_STEP_SIZE);
            targetPosition = new CatzManipulatorPositions(manualHoldTargetPos, -999.0, -999.0);
            m_elevator.cmdUpdateElevator(targetPosition);
          }
      }
      else 
      { //set pwr to 0
          if (isElevatorInManualMode)
          {
            m_elevator.elevatorFullManualCmd(0.0);
          }
      }
  
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

