package frc.robot.subsystems.Arm;

import org.littletonrobotics.junction.AutoLog;
public interface ArmIO 
{
    @AutoLog
    public class ArmIOInputs
    {
        public double armMotorEncoder;
        public boolean isRevLimitSwitchClosed;
        public boolean currentArmControlMode;
    }

    public default void updateInputs(ArmIOInputs inputs) {}
    
    public default void setSelectedSensorPositionIO(double encoderResetPos) {}

    public default void setArmPwrIO(double pwr) {}

    public default void setArmPosEncIO(double targetEncPos) {}
    
    public default void armSetFullExtendPosIO() {}

    
    public default void armSetRetractPosIO() {}

    
    public default void armSetPickupPosIO() {}
}
