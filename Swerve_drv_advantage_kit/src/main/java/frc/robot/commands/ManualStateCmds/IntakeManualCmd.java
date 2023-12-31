// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualStateCmds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.CatzConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.mechMode;
import frc.robot.subsystems.Intake.SubsystemCatzIntake;

public class IntakeManualCmd extends CommandBase {
  SubsystemCatzIntake intake = SubsystemCatzIntake.getInstance();
  Supplier<Double> supplierWristPwr;
  Supplier<Boolean> supplierManualMode;

  
  
  private final double SOFT_LIMIT_FORWARD = 0.0; 
  private final double SOFT_LIMIT_REVERSE = -8900.0; 
  private final double MANUAL_HOLD_STEP_SIZE = 1.5;       
  private final double WRIST_MAX_PWR = 0.3;

  private double   targetPowerCmd = 0.0;
  /** Creates a new IntakeManualCmd. */
  public IntakeManualCmd(Supplier<Double> wristPwr, Supplier<Boolean> supplierManualMode) 
  {
    this.supplierWristPwr = wristPwr;
    this.supplierManualMode = supplierManualMode;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.intakeControlMode = mechMode.AutoMode;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    boolean fullManualModeCmd = supplierManualMode.get();
    double wristPwrCmd   = supplierWristPwr.get();
  
      if(Math.abs(wristPwrCmd) >= 0.1)//if we are apply wrist power manually
      {
          if (fullManualModeCmd) //check if in full manual mode
          {
            targetPowerCmd = wristPwrCmd * WRIST_MAX_PWR;   
            intake.wristFullManual(targetPowerCmd); 
          }
          else //manual holding
          {
            intake.manualHoldingFunction(wristPwrCmd);
          }
      }
      else //Manual power is OFF
      {
          if(fullManualModeCmd)//if we are still in manual mode and want to hold intake in place
          {
              targetPowerCmd = 0.0;
              intake.wristFullManual(targetPowerCmd);
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
