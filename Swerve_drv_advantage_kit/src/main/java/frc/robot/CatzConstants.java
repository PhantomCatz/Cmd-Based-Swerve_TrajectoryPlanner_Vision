/***
 * CatzConstants
 * @version 1.0
 * @author Kynam Lenghiem
 * 
 * This class is where reusable constants are defined
 ***/

package frc.robot;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Utils.CatzManipulatorPositions;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class CatzConstants {
  public static final boolean tuningMode = false;
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running on a real robot with phoenix pro */
    REAL_WITH_PRO,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
  //--------------------Alliance color---------------------------
  public static enum AllianceColor {
    BlUE_ALLIANCE,
    RED_ALLIANCE
  }

 public static final class OIConstants {
  public static final int kDriverControllerPort = 0;

  public static final int kDriverYAxis = 1;
  public static final int kDriverXAxis = 0;
  public static final int kDriverRotAxis = 4;
  public static final int kDriverFieldOrientedButtonIdx = 1;

  public static final double kDeadband = 0.1;
}

public static final class ManipulatorPoseConstants
{
  public static final CatzManipulatorPositions SCORE_HIGH_CONE = new CatzManipulatorPositions(ElevatorConstants.ELEVATOR_POS_ENC_CNTS_HIGH,
                                                                                              ArmConstants.POS_ENC_INCH_EXTEND,
                                                                                              IntakeConstants.SCORE_CONE_HIGH_ENC_POS_TELOP);
  public static final CatzManipulatorPositions SCORE_HIGH_CUBE = new CatzManipulatorPositions(ElevatorConstants.ELEVATOR_POS_ENC_CNTS_HIGH,
                                                                                              ArmConstants.POS_ENC_INCH_EXTEND,
                                                                                              IntakeConstants.SCORE_CUBE_ENC_POS);

  public static final CatzManipulatorPositions SCORE_MID_CONE = new CatzManipulatorPositions(ElevatorConstants.ELEVATOR_POS_ENC_CNTS_MID_CONE,
                                                                                              ArmConstants.POS_ENC_CNTS_RETRACT,
                                                                                              IntakeConstants.SCORE_CONE_MID_ENC_POS);
  public static final CatzManipulatorPositions SCORE_MID_CUBE = new CatzManipulatorPositions(ElevatorConstants.ELEVATOR_POS_ENC_CNTS_MID_CUBE,
                                                                                              ArmConstants.POS_ENC_INCH_RETRACT,
                                                                                              IntakeConstants.SCORE_CUBE_ENC_POS);                                                                                            
                                                                                              
  public static final CatzManipulatorPositions SCORE_LOW_CONE = new CatzManipulatorPositions(ElevatorConstants.ELEVATOR_POS_ENC_CNTS_LOW,
                                                                                              ArmConstants.POS_ENC_CNTS_PICKUP,
                                                                                              IntakeConstants.SCORE_CONE_LOW_ENC_POS);
  public static final CatzManipulatorPositions SCORE_LOW_CUBE = new CatzManipulatorPositions(ElevatorConstants.ELEVATOR_POS_ENC_CNTS_LOW,
                                                                                              ArmConstants.POS_ENC_INCH_PICKUP,
                                                                                              IntakeConstants.SCORE_CUBE_ENC_POS); 

  public static final CatzManipulatorPositions STOW = new CatzManipulatorPositions(ElevatorConstants.ELEVATOR_POS_ENC_CNTS_LOW,
                                                                                              ArmConstants.POS_ENC_CNTS_RETRACT,
                                                                                              IntakeConstants.STOW_ENC_POS);

  public static final CatzManipulatorPositions PICKUP_CONE_GROUND = new CatzManipulatorPositions(ElevatorConstants.ELEVATOR_POS_ENC_CNTS_LOW,
                                                                                                  ArmConstants.POS_ENC_CNTS_PICKUP,
                                                                                                  IntakeConstants.INTAKE_CONE_ENC_POS_GROUND);
  public static final CatzManipulatorPositions PICKUP_CUBE_GROUND = new CatzManipulatorPositions(ElevatorConstants.ELEVATOR_POS_ENC_CNTS_LOW,
                                                                                                  ArmConstants.POS_ENC_INCH_PICKUP,
                                                                                                  IntakeConstants.INTAKE_CUBE_ENC_POS);

  public static final CatzManipulatorPositions PICKUP_CONE_SINGLE = new CatzManipulatorPositions(ElevatorConstants.ELEVATOR_POS_ENC_CNTS_SINGLE_PICKUP,
                                                                                                  ArmConstants.POS_ENC_INCH_PICKUP,
                                                                                                  IntakeConstants.INTAKE_CONE_ENC_POS_SINGLE_UPRIGHT); 
}

//--------------------------------------Drivetrain-------------------------------
      //----------------------Catz auton Constants---------------------------
      public static final class DriveConstants {

        public static final double LT_FRNT_OFFSET = 0.0112305378; //this one changed
        public static final double LT_BACK_OFFSET = 0.0446386386;
        public static final double RT_BACK_OFFSET = 0.2591109064;
        public static final double RT_FRNT_OFFSET = 0.0363121009;

        public static final int LT_FRNT_DRIVE_ID = 1;
        public static final int LT_BACK_DRIVE_ID = 3;//TBD put in constants
        public static final int RT_BACK_DRIVE_ID = 22;
        public static final int RT_FRNT_DRIVE_ID = 7;
        
        public static final int LT_FRNT_STEER_ID = 2;
        public static final int LT_BACK_STEER_ID = 4;
        public static final int RT_BACK_STEER_ID = 6;
        public static final int RT_FRNT_STEER_ID = 8;
    
        public static final int LT_FRNT_ENC_PORT = 9;
        public static final int LT_BACK_ENC_PORT = 6;
        public static final int RT_BACK_ENC_PORT = 7;
        public static final int RT_FRNT_ENC_PORT = 8;

        public static final int     CURRENT_LIMIT_AMPS            = 55;
        public static final int     CURRENT_LIMIT_TRIGGER_AMPS    = 55;
        public static final double  CURRENT_LIMIT_TIMEOUT_SECONDS = 0.5;
        public static final boolean ENABLE_CURRENT_LIMIT          = true;
    
        public static final int     STEER_CURRENT_LIMIT_AMPS      = 30;
        public static final double  NEUTRAL_TO_FULL_SECONDS       = 0.1;
        public static final double  VEL_FF                        = 1.5;

        public static final Pose2d initPose = new Pose2d(0, 0, Rotation2d.fromDegrees(180));
        private static final double MODULE_DISTANCE_FROM_CENTER = 0.298;


        //not following the original coordinate system since the robot coordinate system is inverted
        private static final Translation2d SWERVE_LEFT_FRONT_LOCATION  = new Translation2d(MODULE_DISTANCE_FROM_CENTER, MODULE_DISTANCE_FROM_CENTER);
        private static final Translation2d SWERVE_LEFT_BACK_LOCATION   = new Translation2d(-MODULE_DISTANCE_FROM_CENTER, MODULE_DISTANCE_FROM_CENTER);
        private static final Translation2d SWERVE_RIGHT_BACK_LOCATION  = new Translation2d(-MODULE_DISTANCE_FROM_CENTER, -MODULE_DISTANCE_FROM_CENTER);
        private static final Translation2d SWERVE_RIGHT_FRONT_LOCATION = new Translation2d(MODULE_DISTANCE_FROM_CENTER, -MODULE_DISTANCE_FROM_CENTER);
        
        // calculates the orientation and speed of individual swerve modules when given the motion of the whole robot
        public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
            SWERVE_LEFT_FRONT_LOCATION,
            SWERVE_LEFT_BACK_LOCATION,
            SWERVE_RIGHT_BACK_LOCATION,
            SWERVE_RIGHT_FRONT_LOCATION
        );
        

        public static final double MAX_SPEED = 2.0; // meters per second
        public static final double MAX_ANGSPEED_RAD_PER_SEC = 6.0; // radians per second

        public static final double SDS_L1_GEAR_RATIO = 8.14;       //SDS mk4i L1 ratio reduction
        public static final double SDS_L2_GEAR_RATIO = 6.75;       //SDS mk4i L2 ratio reduction
        
        public static final double DRVTRAIN_WHEEL_DIAMETER_METERS             = 0.095;
        public static final double DRVTRAIN_WHEEL_CIRCUMFERENCE        = (Math.PI * DRVTRAIN_WHEEL_DIAMETER_METERS);

        //uses a trapezoidal velocity/time graph enforced with a PID loop
        private static ProfiledPIDController autoTurnPIDController
                = new ProfiledPIDController(4, 0, 0, new TrapezoidProfile.Constraints(MAX_ANGSPEED_RAD_PER_SEC, MAX_ANGSPEED_RAD_PER_SEC));
            //TBD need to validated
        static{
            autoTurnPIDController.enableContinuousInput(-Math.PI, Math.PI); //offset clamped between these two values
            autoTurnPIDController.setTolerance(Math.toRadians(0.1)); //tolerable error
        }
            //TBD need to validated
        // calculates target chassis motion when given current position and desired trajectory
        public static final HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
            new PIDController(2, 0, 0), // PID values for x offset
            new PIDController(2, 0, 0), // PID values for y offset
            autoTurnPIDController // PID values for orientation offset
        );

        // calculates target chassis motion when given current position and desired trajectory
        public static final PPHolonomicDriveController ppholonomicDriveController = new PPHolonomicDriveController(
        new PIDConstants(0.35, 0, 0), // PID values for x offset
        new PIDConstants(0.35, 0, 0), // PID values for rotation 
        MAX_SPEED,
        MODULE_DISTANCE_FROM_CENTER
        );
    }

  //-----------------------------Intake------------------------------------------

  public static final class IntakeConstants 
  {
    // ----------------------------------------------------------------------------------------------
    // Wrist encoder & Position Values
    // ----------------------------------------------------------------------------------------------

    private static final double ENC_TO_INTAKE_GEAR_RATIO = (46.0 / 18.0)* (32.0 / 10.0);
    public static final double WRIST_CNTS_PER_DEGREE = (2096.0 * ENC_TO_INTAKE_GEAR_RATIO) / 360.0;

    public static final double MANUAL_HOLD_STEP_SIZE = 2.0;

    // Just above parallel with the ground
    public static final double CENTER_OF_MASS_OFFSET_DEG = 177.0;
    public static final double WRIST_ABS_ENC_OFFSET_DEG = 0.0; // Set to make stow pos equal to 0
    public static final double WRIST_ABS_ENC_OFFSET = WRIST_ABS_ENC_OFFSET_DEG * WRIST_CNTS_PER_DEGREE;// -989.0; //Negative
                                                                                                 // value means abs enc
                                                                                                 // 0 is above intake
                                                                                                 // angle 0

    public static final double STOW_ENC_POS = -20.0;
    public static final double STOW_CUTOFF = -30.232;

    public static final double INTAKE_CUBE_ENC_POS = -140.000;
    public static final double INTAKE_CONE_ENC_POS_GROUND = -170.524;
    public static final double INTAKE_CONE_ENC_POS_SINGLE = -100.400;

    public static final double INTAKE_CONE_ENC_POS_SINGLE_UPRIGHT = -80.000;                                                                                        

    public static final double SCORE_CUBE_ENC_POS = -90.000;

    public static final double SCORE_CONE_HIGH_ENC_POS_AUTON = -140.000;
    public static final double SCORE_CONE_HIGH_ENC_POS_TELOP = -139.000;

    public static final double SCORE_CONE_MID_ENC_POS = -170.000;
    public static final double SCORE_CONE_LOW_ENC_POS = -130.00;

    public static final double SOFT_LIMIT_FORWARD = -160.0; 
    public static final double SOFT_LIMIT_REVERSE = -8900.0;

    public static final double GROSS_kP = 0.002472;// 0.0070;//0.00009;
    public static final double GROSS_kI = 0.0;// 000040;
    public static final double GROSS_kD = 0.000291;// 0.000007;

    public static final double FINE_kP = 0.005234;// 0.00009;
    public static final double FINE_kI = 0.0;// 000008;
    public static final double FINE_kD = 0.000291;// 0.000291;//0.000007;

    public static final double MAX_GRAVITY_FF = 0.07; // 0.055

    public final double ERROR_INTAKE_THRESHOLD_DEG = 5.0;
    public final double PID_FINE_GROSS_THRESHOLD_DEG = 17.0;

  }


//-------------------------------Elevator-------------------------------------------------------

public static final class ElevatorConstants
{
    public final static double ELEVATOR_MAX_MANUAL_SCALED_POWER = 0.7;

    public final double ELEVATOR_MANUAL_CONTROL_DEADBAND = 0.1;

    public final double ELEVATOR_MANUAL_CONTROL_PWR_OFF = 0.0;

    public final double ELEVATOR_MANUAL_HOLD_STEP_SIZE = 10000.0; //5000.0;


    //constants for calc encoder to inch

    private static final double ELEVATOR_FIRST_GEAR1 = 13;
    private static final double ELEVATOR_FIRST_GEAR2 = 48;
    private static final double ELEVATOR_FIRST_GEAR_RATIO = ELEVATOR_FIRST_GEAR2/ELEVATOR_FIRST_GEAR1;

    private static final double ELEVATOR_HTD1 = 15;
    private static final double ELEVATOR_HTD2 = 30;
    private static final double ELEVATOR_HTD_RATIO = ELEVATOR_HTD2/ELEVATOR_HTD1;

    private static final double ELEVATOR_SECOND_GEAR1 = 28;
    private static final double ELEVATOR_SECOND_GEAR2 = 24;
    private static final double ELEVATOR_SECOND_GEAR_RATIO = ELEVATOR_SECOND_GEAR2/ELEVATOR_SECOND_GEAR1;

    private static final double ELEVATOR_FINAL_RATIO = ELEVATOR_FIRST_GEAR_RATIO*ELEVATOR_HTD_RATIO*ELEVATOR_SECOND_GEAR_RATIO;

    private static final double ELEVATOR_CNTS_TO_REV = 2048/1;

    private static final double ELEVATOR_SPROKET_DIAMETER = 1.751;
    private static final double ELEVATOR_SPROKET_CIRCUMFERENCE = ELEVATOR_SPROKET_DIAMETER*Math.PI;

    private static final double ELEVATOR_INCHES_TO_COUNTS_CONVERSTION_FACTOR = ((ELEVATOR_CNTS_TO_REV*ELEVATOR_FINAL_RATIO)/ELEVATOR_SPROKET_CIRCUMFERENCE);


    //----------------------------------------------------------------------------------------------
    //  Elevator Position Values
    //----------------------------------------------------------------------------------------------

    public static final double ELEVATOR_POS_ENC_INCH_LOW  = 0.0;
    public static final double ELEVATOR_POS_ENC_INCH_MID_CONE  = 39.616;
    public static final double ELEVATOR_POS_ENC_INCH_MID_CUBE  = 26;
    public static final double ELEVATOR_POS_ENC_INCH_HIGH = 47.187;

    public static final double ELEVATOR_POS_ENC_CNTS_LOW  = ELEVATOR_POS_ENC_INCH_LOW * ELEVATOR_INCHES_TO_COUNTS_CONVERSTION_FACTOR;
    public static final double ELEVATOR_POS_ENC_CNTS_MID_CONE  = ELEVATOR_POS_ENC_INCH_MID_CONE * ELEVATOR_INCHES_TO_COUNTS_CONVERSTION_FACTOR;//91000.0;// needs to be lower...too high
    // private final double POS_ENC_CNTS_MID_CUBE  = POS_ENC_INCH_MID_CUBE * INCHES_TO_COUNTS_CONVERSTION_FACTOR;
    public static final double ELEVATOR_POS_ENC_CNTS_MID_CUBE = 50000.0;
    public static final double ELEVATOR_POS_ENC_CNTS_SINGLE_PICKUP = 27739;

    public static final double ELEVATOR_POS_ENC_CNTS_HIGH = ELEVATOR_POS_ENC_INCH_HIGH * ELEVATOR_INCHES_TO_COUNTS_CONVERSTION_FACTOR;//111200.0;

    public static final double ELEVATOR_KP_LOW = 0.03;
    public static final double ELEVATOR_KI_LOW = 0.0002;
    public static final double ELEVATOR_KD_LOW = 0.001;

    public static final double ELEVATOR_KP_MID = 0.083;
    public static final double ELEVATOR_KI_MID = 0.0002;
    public static final double ELEVATOR_KD_MID = 0.0;

    public static final double ELEVATOR_KP_HIGH = ELEVATOR_KP_MID;
    public static final double ELEVATOR_KI_HIGH = ELEVATOR_KI_MID;
    public static final double ELEVATOR_KD_HIGH = ELEVATOR_KD_MID;

    public static final double ELEVATOR_ARM_ENCODER_THRESHOLD = 35000.0;

    public static final double ELEVATOR_HOLDING_FF = 0.044;


    public static final double ELEVATOR_CLOSELOOP_ERROR_THRESHOLD_LOW = 50; 
    // private final double CLOSELOOP_ERROR_THRESHOLD_HIGH_MID = 300; 
    public static final double ELEVATOR_CLOSELOOP_ERROR_THRESHOLD_HIGH_MID = 225; 
}


    //-----------------------------------ARM---------------------------------------
    public static final class ArmConstants
    {
      //gear ratio
      private static final double ARM_VERSA_RATIO  = 7.0/1.0;

      private static final double PUILEY_1      = 24.0;
      private static final double PUILEY_2      = 18.0;
      private static final double PUILEY_RATIO  = PUILEY_1 / PUILEY_2;
        
      private static final double ARM_FINAL_RATIO   = ARM_VERSA_RATIO * PUILEY_RATIO;
      private static final double FINAL_CIRCUMFERENCE = 3.54; 

      private static final double CNTS_OVER_REV = 2048.0 / 1.0;

      private static final double CNTS_PER_INCH_CONVERSION_FACTOR = CNTS_OVER_REV/FINAL_CIRCUMFERENCE;
    
      //Encoder Positions
      private static final double POS_ENC_INCH_RETRACT = 0.0;
      private static final double POS_ENC_INCH_EXTEND = 8.157;
      private static final double POS_ENC_INCH_PICKUP = 4.157;
    
      public static final double POS_ENC_CNTS_RETRACT  = POS_ENC_INCH_RETRACT * CNTS_PER_INCH_CONVERSION_FACTOR;// 0.0
      public static final double POS_ENC_CNTS_EXTEND  = POS_ENC_INCH_EXTEND * CNTS_PER_INCH_CONVERSION_FACTOR; //44000
      public static final double POS_ENC_CNTS_PICKUP = POS_ENC_INCH_PICKUP * CNTS_PER_INCH_CONVERSION_FACTOR; //22000
    }
  
}
