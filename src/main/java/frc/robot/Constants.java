// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /**
   * CAN bus IDs
   */
  public static class CanID {
    //Swerve drive modules
    public static final int SWERVE_MODULE_FRONT_LEFT_DRIVE_MOTOR = 21;
    public static final int SWERVE_MODULE_FRONT_LEFT_STEER_MOTOR = 20;
    public static final int SWERVE_MODULE_FRONT_LEFT_STEER_ENCODER = 1;
    public static final int SWERVE_MODULE_FRONT_RIGHT_DRIVE_MOTOR = 11;
    public static final int SWERVE_MODULE_FRONT_RIGHT_STEER_MOTOR = 10;
    public static final int SWERVE_MODULE_FRONT_RIGHT_STEER_ENCODER = 2;
    public static final int SWERVE_MODULE_BACK_LEFT_DRIVE_MOTOR = 16;
    public static final int SWERVE_MODULE_BACK_LEFT_STEER_MOTOR = 17;
    public static final int SWERVE_MODULE_BACK_LEFT_STEER_ENCODER = 4;
    public static final int SWERVE_MODULE_BACK_RIGHT_DRIVE_MOTOR = 14;
    public static final int SWERVE_MODULE_BACK_RIGHT_STEER_MOTOR = 15;
    public static final int SWERVE_MODULE_BACK_RIGHT_STEER_ENCODER = 3;
  }

  /**
   * Current limits - measure in Amps
   */
  public static class CurrentLimit {
    public static class SparkMax {
      public static final int SMART_DRIVE = 30;
      public static final int SMART_STEER = 40;
      public static final int SECONDARY_DRIVE = 80;
      public static final int SECONDARY_STEER = 80;
    }

    public static class Neo {
      public static final int SMART = 60;
      public static final int SECONDARY = 80;
    }
  }

  /**
   * Robot physical constraints (max velocity, max angular velocity, SwerveDriveKinematics, etc.)
   */
  public static class Kinematics {
    /* Robot mass in Lbs. */
    public static final double MASS = Units.lbsToKilograms(145.0); 

    //TODO: MJR verify these measurements

    /* Robot width in Inches */
    public static final double WIDTH = Units.inchesToMeters(24.0);

    /* Robot width in Inches WITH bumpers on */
    public static final double WIDTH_WITH_BUMPERS = Units.inchesToMeters(24.0);

    /* Robot length in Inches */
    public static final double LENGTH = Units.inchesToMeters(29.5);

    /* Robot length in Inches WITH bumpers on */
    public static final double LENGTH_WITH_BUMPERS = Units.inchesToMeters(29.5);

    /**
     * The left-to-right distance between the drivetrain wheels
     * <p>
     * Should be measured from center to center
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.4445;

    /**
     * The front-to-back distance between the drivetrain wheels
     * <p>
     * Should be measured from center to center
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.4445;

    //TODO: MJR - is this calc wrong?  Should it instead be  Math.sqrt(DRIVETRAIN_TRACKWIDTH_METERS * DRIVETRAIN_TRACKWIDTH_METERS * 2) ??
    // confirm/compare with empirical measurement from center of robot to center of wheel
    public static final double DRIVETRAIN_BASE_RADIUS = Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS, DRIVETRAIN_WHEELBASE_METERS) / 2.;

    //SDS Mk4 L3 gear ratios - equivalent to 6.12:1 overall ratio
    public static final double DRIVE_GEAR_RATIO = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0); //Note: (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0) == 1.0/6.12

    /** Conversion between motor rotations and drive meters */
    public static final double DRIVE_POSITION_CONVERSION = Math.PI * Units.inchesToMeters(4.0) * DRIVE_GEAR_RATIO;  
        
    /** Conversion between motor rotations per minute and drive meters per seconds */
    public static final double DRIVE_VELOCITY_CONVERSION = DRIVE_POSITION_CONVERSION / 60.;

    /** Conversion between motor rotations and steer degrees */
    public static final double STEER_POSITION_CONVERSION = 360. * (1. / 12.8); // 12.8:1 gear ratio

    /** Conversion between motor rotations per minute and steer degrees per seconds */
    public static final double STEER_VELOCITY_CONVERSION = STEER_POSITION_CONVERSION / 60.;

    /**
     * The maximum linear velocity of the robot in meters per second. This is a
     * measure of how fast the robot can move linearly. Calculated using the
     * emprical free speed velocity of a NEO. (5676 RPM)
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.089;

    /**
     * The maximum angular velocity of the robot in radians per second. This is a
     * measure of how fast the robot can rotate in place.
     */
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND
            / Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2., DRIVETRAIN_WHEELBASE_METERS / 2.);

    /** Max velocity while following a trajectory. Meters per second */
    public static final double MAX_TRAJECTORY_VELOCITY = 3.;

    /**
     * Max acceleration while following a trajectory. Meters per second per second
     */
    public static final double MAX_TRAJECTORY_ACCELERATION = 2.;

    /**
     * Used  to convert desired chassis velocity into individual swerve smodule states
     * See https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#robot-drive-kinematics
     */
    public static final SwerveDriveKinematics SWERVE_KINEMATICS = new SwerveDriveKinematics(
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2., DRIVETRAIN_WHEELBASE_METERS / 2.),
            new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2., -DRIVETRAIN_WHEELBASE_METERS / 2.),
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2., DRIVETRAIN_WHEELBASE_METERS / 2.),
            new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2., -DRIVETRAIN_WHEELBASE_METERS / 2.));
  }

  public static class PathPlanner {
    public static final double rotation_P = 1.5;
    public static final double rotation_I = 0.0;
    public static final double rotation_D = 0.0;

    public static final double translation_P = 1.0;
    public static final double translation_I = 0.0;
    public static final double translation_D = 0.0;
  }

  public static class Vision {
    public static final boolean VISION_ENABLED = true;
    public static final String ARDUCAM_MODEL = "OV9281";
    public static final double POSE_AMBIGUITY_CUTOFF = 0.05;  //TODO: test and adjust this value if necessary (photon docs suggest using 0.2)
    public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
    public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
    public static final double NOISY_DISTANCE_METERS = 2.5;  //distance beyond which vision measurements are noisy
    public static final double DISTANCE_CUTOFF = 4.0;  //Tag readings beyond this distance (in meters) will be considered invalid
    public static final double DISTANCE_WEIGHT = 7;
    public static final int TAG_PRESENCE_WEIGHT = 10;

    /**
     * Standard deviations for vision measurements. Increase these numbers to trust your
     * vision measurements less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = MatBuilder.fill(Nat.N3(), Nat.N1(),
        // if these numbers are less than one, multiplying will do bad things
        1, // x
        1, // y
        1 * Math.PI // theta
    );
    
    /**
     * Unique camera names, usable in PhotonCamera instances
     */
    public static class CameraName {
      //Note: these names are set in hardware via https://docs.arducam.com/UVC-Camera/Serial-Number-Tool-Guide/
      public static final String CAMERA_1 = "Arducam_OV9281_USB_Camera-1";
      public static final String CAMERA_2 = "Arducam_OV9281_USB_Camera-2";
      public static final String CAMERA_3 = "Arducam_OV9281_USB_Camera-3";
      public static final String CAMERA_4 = "Arducam_OV9281_USB_Camera-4";
    }

    /**
     * Mounting position of the cameras on the Robot
     */
    public static class CameraPose {
      //TODO: MJR mount cameras on robot and define these values using edu.wpi.first.math.util.Units.inchesToMeters() and Units.degreesToRadians()
      //Note: these are robot to camera poses (position from center of robot to camera lens) - see also edu.wpi.first.math.ComputerVisionUtil.objectToRobotPose()
      //In transform3d - Translation3d values: x+ = forward, y+ = left, z+ = up, Rotation3d is rotation around the transform3d axes
      // https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
      // https://docs.wpilib.org/en/stable/_images/drive-yaw-pitch-roll.svg
      //Note: these values can be visualized in AdvantageScope if published over NetworkTables - https://github.com/Mechanical-Advantage/AdvantageScope/blob/main/docs/tabs/3D-FIELD.md
      //example - https://github.com/Mechanical-Advantage/RobotCode2024/blob/main/src/main/java/org/littletonrobotics/frc2024/subsystems/apriltagvision/AprilTagVisionConstants.java#L30
      public static final Transform3d CAMERA_1 = new Transform3d(0, 0 , 0, new Rotation3d(0, 0, 0));
      public static final Transform3d CAMERA_2 = new Transform3d(0, 0 , 0, new Rotation3d(0, 0, 0));
      public static final Transform3d CAMERA_3 = new Transform3d(0, 0 , 0, new Rotation3d(0, 0, 0));
      public static final Transform3d CAMERA_4 = new Transform3d(0, 0 , 0, new Rotation3d(0, 0, 0));
    }
  }

  public static class Game {
    //Note: field layout values can be obtained by examining the AprilTagFieldLayout .json file for the game
    public static final double FIELD_LENGTH_METERS = 16.541;  //x in field drawings (from 2024 game - update for 2025 if necessary)
    public static final double FIELD_WIDTH_METERS = 8.211;  //y in field drawings (from 2024 game - update for 2025 if necessary)
    //These are buffers to accomodate for margin of error
    public static final double FIELD_POSE_XY_ERROR_MARGIN_METERS = Units.inchesToMeters(1.0);
    public static final double FIELD_POSE_THETA_ERROR_MARGIN_RADIANS = Units.degreesToRadians(2.0);
  }

  public static class Shuffleboard {
    public static final ShuffleboardTab COMPETITION_TAB = edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab("Competition");
    public static final ShuffleboardTab DIAG_TAB = edu.wpi.first.wpilibj.shuffleboard.Shuffleboard.getTab("diag");
  }
}
