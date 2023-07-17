package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO 
{
 @AutoLog
 public static class ModuleIOInputs
 {
    public double gyroAngle = 0.0;
    public double driveMtrVelocity = 0.0;
    public double driveMtrSensorPosition = 0.0;
    public double magEncoderValue = 0.0;
    public double driveAppliedVolts = 0.0;
    public double steerAppliedVolts = 0.0;
    public double driveMtrPercentOutput = 0.0;

 }

 /** Updates the set of loggable inputs. */
 public default void updateInputs(ModuleIOInputs inputs) {}

 public default void setDrivePwrPercentIO(double drivePwrPercent) {}

 public default void setSteerPwrIO(double SteerPwr) {}

 public default void setSteerCoastModeIO() {}

 public default void setSteerBrakeModeIO() {}

 public default void resetDrvSensorPositionIO() {}

 public default void reverseDriveIO(Boolean enable) {}

 public default void setDriveSimPwrIO(double volts) {}

 public default void setSteerSimPwrIO(double volts) {}

 public default void resetMagEncoderIO() {}
}