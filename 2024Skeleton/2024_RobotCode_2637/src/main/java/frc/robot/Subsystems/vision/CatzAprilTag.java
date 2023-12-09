package frc.robot.Subsystems.vision;
    
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CatzAprilTag {

    private static CatzAprilTag instance = null;

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    private final double METER_TO_INCH = 39.37;
    private final int REQUIRED_ARRAY_LENGTH = 6;
    private final int NUMBER_OF_LINEAR_DIMENSIONS = 3;
    private static final double DISX_TAG_TO_TAG = 570.32; //distance form this side apriltag to the other side apriltag

    private final int POS_X_INDEX = 0;
    private final int POS_Y_INDEX = 1;
    private final int ROT_Z_INDEX = 5;
    
    private static final int BLUE_ALLIANCE = 0;
    private static final int RED_ALLIANCE = 1;

    private double[] botPose = null;

    private int alliance;

    private CatzAprilTag(int allianceColor)
    {
        alliance = allianceColor;
    }

    //grab botpos from limelight load into this class, load INVALID_POSE_ENTY if no apriltag is seeing
    public void botPoseUpdate()
    {
        if(aprilTagInView() && table.getEntry("botpose").getDoubleArray(botPose).length == REQUIRED_ARRAY_LENGTH)
        {
            for(int i = 0; i < REQUIRED_ARRAY_LENGTH; i++)
            {
                if (i  < NUMBER_OF_LINEAR_DIMENSIONS && alliance == RED_ALLIANCE)
                {
                    botPose[i] = table.getEntry("botpose").getDoubleArray(botPose)[i]*METER_TO_INCH;
                }
                else if(i  < NUMBER_OF_LINEAR_DIMENSIONS)
                {
                    botPose[i] = -table.getEntry("botpose").getDoubleArray(botPose)[i]*METER_TO_INCH;
                }
                else if (i == ROT_Z_INDEX && alliance == RED_ALLIANCE)
                {
                    botPose[i] = table.getEntry("botpose").getDoubleArray(botPose)[i]-180.0;
                    botPose[i] = -Math.signum(botPose[i])*180.0 - (botPose[i] % 180.0);
                }
                else
                {
                    botPose[i] = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(botPose)[i];
                }
            }
        }
        else
        {
            botPose = null;
        }
    }

    public boolean aprilTagInView()
    {
        return (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1);
    }

    public Pose2d getLimelightBotPose()
    {
        botPoseUpdate();
        return new Pose2d(botPose[POS_X_INDEX],botPose[POS_Y_INDEX],Rotation2d.fromDegrees(botPose[ROT_Z_INDEX]));
    }

    //return the distance from center of robot to apriltag
    public double disToTag()
    {
        return (DISX_TAG_TO_TAG/2-Math.abs(botPose[POS_X_INDEX]));
    }

    public void smartDashboardAprilTag() {
        SmartDashboard.putNumber("Limielightx", x);
        SmartDashboard.putNumber("Limielighty", y);
        SmartDashboard.putNumber("Limielighta", area);
    }

    public static CatzAprilTag getInstance()
    {
        if(instance == null)
        {
            instance = new CatzAprilTag(BLUE_ALLIANCE);
        }
        return instance;
    }


}
