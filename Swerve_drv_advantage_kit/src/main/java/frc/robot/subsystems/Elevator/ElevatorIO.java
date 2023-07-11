package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO 
{
    @AutoLog
    public class ElevatorIOInputs
    {

    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void elevatorManualIO(double finalMtrPower) {}

    public default void elevatorSetToLowPosIO() {}

    public default void elevatorSetToMidPosConeIo() {}
    
    

}
