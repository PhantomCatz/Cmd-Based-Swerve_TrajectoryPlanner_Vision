// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class CatzConstants {
  public static final Mode currentMode = Mode.SIM;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
  //--------------------State machine Constants---------------------------
  public static final int GP_NONE = 0;
  public static final int GP_CUBE = 1;
  public static final int GP_CONE = 2;

  //--------------------


  //----------------------Catz auton Constants---------------------------
  private static final double MODULE_DISTANCE_FROM_CENTER = 0.2984;

  private static final Translation2d SWERVE_LEFT_FRONT_LOCATION  = new Translation2d(MODULE_DISTANCE_FROM_CENTER,MODULE_DISTANCE_FROM_CENTER);
  private static final Translation2d SWERVE_LEFT_BACK_LOCATION   = new Translation2d(MODULE_DISTANCE_FROM_CENTER, -MODULE_DISTANCE_FROM_CENTER);
  private static final Translation2d SWERVE_RIGHT_FRONT_LOCATION = new Translation2d(-MODULE_DISTANCE_FROM_CENTER, MODULE_DISTANCE_FROM_CENTER);
  private static final Translation2d SWERVE_RIGHT_BACK_LOCATION  = new Translation2d(-MODULE_DISTANCE_FROM_CENTER, -MODULE_DISTANCE_FROM_CENTER);

  public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
      SWERVE_LEFT_FRONT_LOCATION,
      SWERVE_LEFT_BACK_LOCATION,
      SWERVE_RIGHT_FRONT_LOCATION,
      SWERVE_RIGHT_BACK_LOCATION
  );

  public static final double MAX_SPEED = 4.0;

  public static final double SDS_L1_GEAR_RATIO = 8.14;       //SDS mk4i L1 ratio
  public static final double SDS_L2_GEAR_RATIO = 6.75;       //SDS mk4i L2 ratio
  
  public static final double DRVTRAIN_WHEEL_DIAMETER             = 4.0;
  public static final double DRVTRAIN_WHEEL_CIRCUMFERENCE        = (Math.PI * DRVTRAIN_WHEEL_DIAMETER);


  public final double POS_ENC_CNTS_HIGH_EXTEND_THRESHOLD_ELEVATOR = 73000.0;
}
