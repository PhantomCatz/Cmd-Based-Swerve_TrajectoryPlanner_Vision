
package frc.robot.commands;



import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.CatzConstants;
import frc.robot.subsystems.drivetrain.CatzDriveTrainSubsystem;

public class DriveProcCmd extends CommandBase {

  private CatzDriveTrainSubsystem driveTrain = CatzDriveTrainSubsystem.getInstance();
  Supplier<Double> supplierLeftJoyX;
  Supplier<Double> supplierLeftJoyY;
  Supplier<Double> supplierRightJoyX;
  Supplier<Boolean> supplierPwrMode;

  private double steerAngle;
  private double drivePower;
  private double turnPower;
  private double gyroAngle;
  private boolean modifyDrvPwr;

  
  /** Creates a new TeleopDriveCmd. */
  public DriveProcCmd(Supplier<Double> supplierLeftJoyX,
                    Supplier<Double> supplierLeftJoyY,
                    Supplier<Double> supplierRightJoyX,
                    Supplier<Boolean> supplierPwrMode) 
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
        double turnPower = supplierRightJoyX.get();

        ChassisSpeeds chassisSpeeds;
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(leftJoyX, leftJoyY, turnPower, driveTrain.getRotation2d());

        SwerveModuleState[] moduleStates = CatzConstants.DriveConstants.swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        
        driveTrain.setModuleStates(moduleStates);

        modifyDrvPwr = supplierPwrMode.get();

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
