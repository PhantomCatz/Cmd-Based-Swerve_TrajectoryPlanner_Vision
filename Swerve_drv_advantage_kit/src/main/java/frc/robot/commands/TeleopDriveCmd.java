
package frc.robot.commands;



import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.CatzConstants;
import frc.robot.CatzConstants.DriveConstants;
import frc.robot.CatzConstants.OIConstants;
import frc.robot.subsystems.drivetrain.CatzDriveTrainSubsystem;

public class TeleopDriveCmd extends CommandBase {

  private CatzDriveTrainSubsystem driveTrain = CatzDriveTrainSubsystem.getInstance();
  Supplier<Double> supplierLeftJoyX;
  Supplier<Double> supplierLeftJoyY;
  Supplier<Double> supplierRightJoyX;
  Supplier<Double> supplierPwrMode;
  Supplier<Boolean> isFeildOriented;

  
  /** Creates a new TeleopDriveCmd. */
  public TeleopDriveCmd(Supplier<Double> supplierLeftJoyX,
                        Supplier<Double> supplierLeftJoyY,
                        Supplier<Double> supplierRightJoyX,
                        Supplier<Double> supplierPwrMode,
                        Supplier<Boolean> isFeildOriented) 
  {
    this.supplierLeftJoyX = supplierLeftJoyX;
    this.supplierLeftJoyY = supplierLeftJoyY;
    this.supplierRightJoyX = supplierRightJoyX;
    this.supplierPwrMode = supplierPwrMode;
    this.isFeildOriented = isFeildOriented;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
        //obtain realtime joystick inputs with supplier methods
        double xSpeed = supplierLeftJoyX.get();
        double ySpeed = supplierLeftJoyY.get();
        double turningSpeed = supplierRightJoyX.get();

        // Apply deadbands to prevent modules from receiving unintentional pwr
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        //Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (isFeildOriented.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, driveTrain.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        driveTrain.setModuleStates(moduleStates);

        var frontLeftState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(45));
        var backRightState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(45));
        var backLeftState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(45));
        var frontRightState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(45));


        // Convert to chassis speeds
        ChassisSpeeds chassisSpeedss = DriveConstants.swerveDriveKinematics.toChassisSpeeds(
        frontLeftState, frontRightState, backLeftState, backRightState);
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
