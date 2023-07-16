package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    
    @AutoLog
    public class IntakeIOInputs
    {
        public double wristPosEnc;
        public double wristTemp;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void rollersOffIO() {}

    public default void rollersOnIO(double rollerPwr) {}

    public default void intakeManualHoldingIO(double targetHoldingPwr) {}

    public default void intakeSetPercentOuputIO(double setIntakeMtrPwr) {}

    public default void intakeConfigureSoftLimitOverride(boolean enabled) {}


}
