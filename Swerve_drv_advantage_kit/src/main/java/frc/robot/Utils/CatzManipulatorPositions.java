package frc.robot.Utils;

public class CatzManipulatorPositions {
    private double elevatorPosEnc = 0;
    private double armPosEnc = 0;
    private double wristAngleDeg = 0;

    public CatzManipulatorPositions(double elevatorPosEnc, double armPosEnc, double wristAngleDeg) 
    {
        this.elevatorPosEnc = elevatorPosEnc;
        this.armPosEnc = armPosEnc;
        this.wristAngleDeg = wristAngleDeg;
    }

    public double getElevatorPosEnc() 
    {
        return elevatorPosEnc;
    }

    public double getArmPosEnc() 
    {
        return armPosEnc;
    }

    public double getWristAngleDeg() 
    {
        return wristAngleDeg;
    }
}
