package frc.robot.subsystems.vision;
    
import java.util.ArrayList;
import java.util.List;

import javax.lang.model.util.ElementScanner14;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CatzConstants;

public class SubsystemCatzVision extends SubsystemBase {

    private static SubsystemCatzVision instance = null;

    private static final double LOWEST_DISTANCE = Units.feetToMeters(10.0);

    private final VisionIO[] cameras;
    private final VisionIOInputsAutoLogged[] inputs;

    private final List<SubsystemCatzVision.PoseAndTimestamp> results = new ArrayList<>();

    private int acceptableTagID;
    private boolean useSingleTag = false;


    private final int REQUIRED_ARRAY_LENGTH = 6;
    private final int NUMBER_OF_LINEAR_DIMENSIONS = 3;
    private static final double DISX_TAG_TO_TAG = 570.32; //distance form this side apriltag to the other side apriltag


    private SubsystemCatzVision(VisionIO[] cameras) {
        this.cameras = cameras;
        inputs = new VisionIOInputsAutoLogged[cameras.length];
    }

    @Override
    public void periodic() {
        Logger.getInstance().recordOutput("useSingleTag", useSingleTag); //set by driverstation

        // clear results from last periodic
        results.clear();

        for (int i = 0; i < inputs.length; i++) {
            // update and process new inputs for camera
            cameras[i].updateInputs(inputs[i]);
            Logger.getInstance().processInputs("Vision/" + cameras[i].getName() + "/Inputs", inputs[i]);

            //checks for when to process vision
            if (inputs[i].hasTarget
                    && inputs[i].isNew
                    && !DriverStation.isAutonomous()
                    && inputs[i].maxDistance < LOWEST_DISTANCE) 
            {
                if (useSingleTag) {
                    if (inputs[i].singleIDUsed == acceptableTagID) {
                        processVision(i);
                    }
                } else {
                    processVision(i);
                }

            }
        }

        Logger.getInstance().recordOutput("Vision/ResultCount", results.size());
    }

    public void processVision(int cameraNum) {
        // create a new pose based off the new inputs
        Pose2d currentPose = new Pose2d(inputs[cameraNum].x, 
                             inputs[cameraNum].y, 
                             new Rotation2d(inputs[cameraNum].rotation));

        //log data
        Logger.getInstance().recordOutput(cameras[cameraNum].getName() + " pose", currentPose);

        // add the new pose to a list
        results.add(new PoseAndTimestamp(currentPose, inputs[cameraNum].timestamp));
    }

    /**
     * Returns the last recorded pose
     */
    public List<SubsystemCatzVision.PoseAndTimestamp> getVisionOdometry() {
        return results;
    }

    /**
     * Inner class to record a pose and its timestampand 
     */
    public static class PoseAndTimestamp {
        Pose2d pose;
        double timestamp;

        public PoseAndTimestamp(Pose2d pose, double timestamp) {
            this.pose = pose;
            this.timestamp = timestamp;
        }

        public Pose2d getPose() {
            return pose;
        }

        public double getTimestamp() {
            return timestamp;
        }
    }

    public void setUseSingleTag(boolean useSingleTag) {
        setUseSingleTag(useSingleTag, 0);
    }

    public void setUseSingleTag(boolean useSingleTag, int acceptableTagID) {
        this.useSingleTag = useSingleTag;
        this.acceptableTagID = acceptableTagID;
    }

    public void setReferencePose(Pose2d pose) {
        for (VisionIO io : cameras) {
            io.setReferencePose(pose);
        }
    }

    public double getMinDistance(int camera) {
        return inputs[camera].minDistance;
    }

    
    public static SubsystemCatzVision getInstance()
    {
        if(instance == null) {
            instance = new SubsystemCatzVision(new VisionIO[] {
                new VisionIOLimeLight("limelight", CatzConstants.VisionConstants.LIMELIGHT_OFFSET)
            });
        }
        return instance;
    }


}
