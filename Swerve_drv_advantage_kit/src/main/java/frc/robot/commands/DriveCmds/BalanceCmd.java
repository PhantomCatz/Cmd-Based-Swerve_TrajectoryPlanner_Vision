// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCmds;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utils.CatzMathUtils;
import frc.robot.subsystems.drivetrain.CatzDriveTrainSubsystem;

public class BalanceCmd extends CommandBase {
  /** Creates a new BalanceCmd. */
  public static Timer balanceTimer = new Timer();
  public static double prevTime = 0.0;
  public static double time = 0.0;

  public  Boolean startBalance = false;

  public static double prevBalanceAngle = 0.0;
  public static double balanceAngle = 0.0;
  public static double angleRate = 0.0;
  public static double power = 0.0;
  public static double angleTerm = 0.0;
  public static double rateTerm = 0.0;
  public static double powerFinal = 0.0;

  public final double ANG_SLOWBAND = 10.0; 
  public final double ANG_GAIN = 0.008; //0.007
  public final double RATE_GAIN = 0.002; //0.002 //TBD which values are the most optimal?
  public final double MAX_POWER = 0.30;
  public final double BALANCE_THREAD_PERIOD = 0.02;

  private CatzDriveTrainSubsystem driveTrain = CatzDriveTrainSubsystem.getInstance();
  public BalanceCmd() 
  {
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    balanceTimer.reset();
    balanceTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    time = balanceTimer.get();

    balanceAngle = driveTrain.getRollAngle();

    if(prevTime < 0.0)
    {
        angleRate = 0.0;
    } 
    else 
    {
        angleRate = (balanceAngle - prevBalanceAngle)/(time - prevTime);
    }

    // PID without the I
    angleTerm = balanceAngle * ANG_GAIN;
    rateTerm = angleRate * RATE_GAIN;

    power = CatzMathUtils.Clamp(-MAX_POWER, angleTerm + rateTerm, MAX_POWER);

    if(Math.abs(power)< 0.07)
    {
        if(power < 0)
        {
            power = -0.04;
        }
        else if(power > 0)
        {
            power = 0.04;
        }
        
        if(Math.abs(balanceAngle) < 2.0)
        {
            power = 0.0;
        }
    }

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                            power, 0.0, 0.0, driveTrain.getRotation2d()
                                                              );
    driveTrain.driveRobotRelative(chassisSpeeds);

    prevBalanceAngle = balanceAngle;
    prevTime = time;
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
