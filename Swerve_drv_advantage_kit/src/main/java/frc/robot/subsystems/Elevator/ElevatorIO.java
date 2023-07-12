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

    public default void elevatorConfig_kPIO(double elevatorkP) {}

    public default void elevatorConfig_kIIO(double elevatorkI) {}

    public default void elevatorConfig_kDIO(double elevatorkD) {}


    
    

}
