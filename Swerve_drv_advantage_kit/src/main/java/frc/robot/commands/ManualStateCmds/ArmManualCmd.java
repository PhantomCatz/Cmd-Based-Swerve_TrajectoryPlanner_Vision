// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ManualStateCmds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Utils.CatzManipulatorPositions;
import frc.robot.Utils.CatzSharedDataUtil;
import frc.robot.subsystems.Arm.CatzArmSubsystem;

public class ArmManualCmd extends CommandBase {
  private CatzArmSubsystem arm = CatzArmSubsystem.getInstance();
  private boolean armExtend;
  private boolean armRetract;

  private final double EXTEND_PWR  = 0.2;
  private final double RETRACT_PWR = -0.2;
  private final double MANUAL_CONTROL_PWR_OFF = 0.0;

  
  /** Creates a new ArmManualCmd. */
  public ArmManualCmd(boolean armExtend, boolean armRetract) 
  {
    this.armExtend = armExtend;
    this.armRetract = armRetract;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(armExtend == true)
    {
        arm.setArmPwr(EXTEND_PWR);
    }
    else if(armRetract == true)
    {
        arm.setArmPwr(RETRACT_PWR);
    }
    else if(CatzSharedDataUtil.sharedArmInControlMode)
    {
        arm.setArmPwr(MANUAL_CONTROL_PWR_OFF);
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
