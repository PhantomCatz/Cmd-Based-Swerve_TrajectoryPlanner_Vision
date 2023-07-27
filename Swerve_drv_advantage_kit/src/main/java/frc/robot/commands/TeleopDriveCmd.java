
package frc.robot.commands;



import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.CatzDriveTrainSubsystem;

public class TeleopDriveCmd extends CommandBase {

  private CatzDriveTrainSubsystem driveTrain = CatzDriveTrainSubsystem.getInstance();
  Supplier<Double> supplierLeftJoyX;
  Supplier<Double> supplierLeftJoyY;
  Supplier<Double> supplierRightJoyX;
  Supplier<Double> supplierPwrMode;

  private double steerAngle;
  private double drivePower;
  private double turnPower;
  private double gyroAngle;
  private boolean modifyDrvPwr;

  
  /** Creates a new TeleopDriveCmd. */
  public TeleopDriveCmd(Supplier<Double> supplierLeftJoyX,
                        Supplier<Double> supplierLeftJoyY,
                        Supplier<Double> supplierRightJoyX,
                        Supplier<Double> supplierPwrMode) 
  {
    this.supplierLeftJoyX = supplierLeftJoyX;
    this.supplierLeftJoyY = supplierLeftJoyY;
    this.supplierRightJoyX = supplierRightJoyX;
    this.supplierPwrMode = supplierPwrMode;  

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
        double leftJoyX = supplierLeftJoyX.get();
        double leftJoyY = supplierLeftJoyY.get();
        double rightJoyX = supplierRightJoyX.get();
        double pwrMode = supplierPwrMode.get();

        steerAngle = driveTrain.calcJoystickAngle(leftJoyX, leftJoyY);
        drivePower = driveTrain.calcJoystickPower(leftJoyX, leftJoyY);
        turnPower  = rightJoyX;
        
        gyroAngle  = driveTrain.getGyroAngle();

        
        if(pwrMode > 0.9)
        {
            modifyDrvPwr = true;
        }
        else
        {
            modifyDrvPwr = false;
        }

        if(drivePower >= 0.1)
        {
            
            if(modifyDrvPwr == true)
            {
                drivePower = drivePower * 0.5;
            }
            

            if(Math.abs(turnPower) >= 0.1)
            {
                if(modifyDrvPwr == true)
                {
                    turnPower = turnPower * 0.5;
                }
                driveTrain.translateTurn(steerAngle, drivePower, turnPower, gyroAngle);
                }
               else
            {
                driveTrain.drive(steerAngle, drivePower, gyroAngle);
            }

        }
        else if(Math.abs(turnPower) >= 0.1)
        {
            if(modifyDrvPwr == true)
            {
                turnPower = turnPower * 0.5;
            }
            
            driveTrain.rotateInPlace(turnPower);
            
        }
        else
        {
            driveTrain.setSteerPower(0.0);
            driveTrain.setDrivePower(0.0);
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
