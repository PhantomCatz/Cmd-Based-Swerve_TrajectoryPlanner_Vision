// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualStateCmds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake.CatzIntakeSubsystem;

public class IntakeManualCmd extends CommandBase {
  CatzIntakeSubsystem intake = CatzIntakeSubsystem.getInstance();
  Supplier<Double> supplierWristPwr;
  Supplier<Boolean> supplierManualMode;

  
  
  private final double SOFT_LIMIT_FORWARD = 0.0; //4876  + WRIST_ABS_ENC_OFFSET;  //3887
  private final double SOFT_LIMIT_REVERSE = -8900.0; //-798.0 + WRIST_ABS_ENC_OFFSET; //-1787     //TBD
  private final double MANUAL_HOLD_STEP_SIZE = 1.5;       
  private final double WRIST_MAX_PWR = 0.3;

  private double   targetPowerCmd = 0.0;
  /** Creates a new IntakeManualCmd. */
  public IntakeManualCmd(Supplier<Double> wristPwr, Supplier<Boolean> supplierManualMode) 
  {
    this.supplierWristPwr = wristPwr;
    this.supplierManualMode = supplierManualMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    boolean fullManualModeCmd = supplierManualMode.get();
    double wristPwrCmd   = supplierWristPwr.get();
  
      if(fullManualModeCmd)
      {              
        intake.setPIDEnable(false);  
      }
  
      if(Math.abs(wristPwrCmd) >= 0.1)//if we are apply wrist power manually
      {
          if (intake.getPIDEnabled() == true)//check if in manual holding state
          {
            intake.manualHoldingFunction(wristPwrCmd);
          }
          else //in full manual mode
          {
              targetPowerCmd = wristPwrCmd * WRIST_MAX_PWR;    
              intake.wristSetPercentOuput(targetPowerCmd);
          }
      }
      else //Manual power is OFF
      {
          if(intake.getPIDEnabled() == false)//if we are still in manual mode and want to hold intake in place
          {
              targetPowerCmd = 0.0;
              intake.wristSetPercentOuput(targetPowerCmd);
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
