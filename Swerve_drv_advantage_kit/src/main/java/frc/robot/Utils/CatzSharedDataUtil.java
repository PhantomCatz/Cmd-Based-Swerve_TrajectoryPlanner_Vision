package frc.robot.Utils;

//dedicated class for sharing any variables accross mechanisms or commands
public class CatzSharedDataUtil {
    public static double sharedElevatorEncCnts = -999.0;
    public static double sharedArmEncCnts = -999.0;
    public static double sharedWristEncCnts = -999.0;
    public static double sharedWristTargetAng = -999.0;
    public static boolean sharedArmInControlMode = false;
    public static boolean sharedElevatorInPos = false;
    public static boolean sharedArmInPos    = false;
    public static boolean sharedIntakeInPos = false;
}
