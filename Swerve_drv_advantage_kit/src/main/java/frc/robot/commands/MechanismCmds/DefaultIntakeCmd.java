// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.MechanismCmds;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils.CatzStateUtil;
import frc.robot.Utils.CatzStateUtil.IntakeState;
import frc.robot.subsystems.Intake.CatzIntakeSubsystem;

public class DefaultIntakeCmd extends CommandBase {
  private CatzIntakeSubsystem intake = CatzIntakeSubsystem.getInstance();
  CatzStateUtil.IntakeState currentIntakeState;
  CatzStateUtil.SetMechanismState currentMechanismState;
  Supplier<Double> supplierWristPwr;
  Supplier<Boolean> supplierManualMode;

  private final double SOFT_LIMIT_FORWARD = 0.0; //4876  + WRIST_ABS_ENC_OFFSET;  //3887
  private final double SOFT_LIMIT_REVERSE = -8900.0; //-798.0 + WRIST_ABS_ENC_OFFSET; //-1787     //TBD
  private final double MANUAL_HOLD_STEP_SIZE = 1.5;       
  private final double WRIST_MAX_PWR = 0.3;



  private double   targetPowerCmd = 0.0;
  private double   targetPosDegCmd;



  private int numConsectSamples = 0;
  /** Creates a new DefaultIntakeCmd. */
  public DefaultIntakeCmd() 
  {

    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
   intake.IntakePIDLoop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() 
  {
    return false;
  }
}
