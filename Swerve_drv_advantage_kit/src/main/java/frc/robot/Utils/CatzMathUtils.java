package frc.robot.Utils;

public class CatzMathUtils {
    public static double closestAngle(double startAngle, double targetAngle)
    {
        // get direction
        double error = targetAngle % 360.0 - startAngle % 360.0;

        // convert from -360 to 360 to -180 to 180
        if (Math.abs(error) > 180.0)
        {
            error = -(Math.signum(error) * 360.0) + error;
            //closest angle shouldn't be more than 180 degrees. If it is, use other direction
            if(error > 180.0)
            {
                error -= 360;
            }
        }

        return error;
    }

    public static double velocityCntsToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    public static double velocityCntsToMPS(double velocitycounts, double circumference, double gearRatio){
        double wheelRPM = velocityCntsToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }
}
