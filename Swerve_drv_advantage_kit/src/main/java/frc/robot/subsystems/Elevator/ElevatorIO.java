package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO //tbd change to ifc suffix
{
    @AutoLog
    public class ElevatorIOInputs
    {
        public double elevatorEncoderCnts = 0.0;
        public boolean isRevLimitSwitchClosed;
        public boolean isFwdLimitSwitchClosed;
    }

    public default void updateInputs(ElevatorIOInputs inputsIO) {}

    public default void elevatorManualIO(double finalMtrPower) {}

    public default void elevatorConfig_kPIO(int slotID, double elevatorkP) {}

    public default void elevatorConfig_kIIO(int slotID, double elevatorkI) {}

    public default void elevatorConfig_kDIO(int slotID, double elevatorkD) {}

    public default void elevatorMtrSetPosIO(double setPosition) {}

    public default void configAllowableClosedloopErrorIO(int slotID, double closeloopErrorThreshold) {}

    public default void setSelectedSensorPositionIO(double setNewReadPosition) {}



    
    

}
