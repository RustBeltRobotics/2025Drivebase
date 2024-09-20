package frc.robot.subsystems;

import java.util.List;
import java.util.Map;

import org.photonvision.EstimatedRobotPose;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Vision;

/**
	- Contains majority of logic for driving the robot / controls the 4 SwerveModule instance (one for each wheel)
	- Publishes robot pose and swerve module state data to network tables (for use by AdvantageScope / PathPlanner)
	- Configures Auto chooser for selecting PathPlanner paths for Autonomous mode
	- Reads Gyro readings for robot angle (yaw) and angular velocity (using NavX AHRS - Altitude and Heading Reference System)
	- Updates robot odometry (estimation of location based on sensors) from swerve drive module states and vision system AprilTag readings (Limelight)
	- Handles special drive modes from driver controller (evasion and wheel locking)
 */
public class Drivetrain extends SubsystemBase {

    private String theMove;
    // NavX connected over MXP
    public final AHRS navx;

    private VisionSystem visionSystem;  //set this externally if you want to use vision measurements for odometry

    /**
     * For user to reset zero for "forward" on the robot while maintaining absolute
     * field zero for odometry
     */

    // These are our modules. We initialize them in the constructor.
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    // The speed of the robot in x and y translational velocities and rotational
    // velocity
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    // Boolean statement to control locking the wheels in an X-position
    private boolean wheelsLocked = false;

    private final SwerveDrivePoseEstimator poseEstimator;

    private SwerveModuleState[] states;

    //Shuffleboard
    private static GenericEntry gyroEntry = Constants.Shuffleboard.COMPETITION_TAB.add("Gryoscope Angle", 0)
            .withWidget(BuiltInWidgets.kGyro)
            .withPosition(0, 1)
            .withSize(3, 4)
            .getEntry();

    private static GenericEntry gyroWarningEntry = Constants.Shuffleboard.COMPETITION_TAB.add("!GYRO!", false)
            .withWidget("Boolean Box")
            .withPosition(7, 0)
            .withProperties(Map.of("colorWhenTrue", "red", "colorWhenFalse", "gray"))
            .getEntry();

    //https://docs.wpilib.org/en/stable/docs/software/networktables/networktables-intro.html#networktables-organization
    // networktables publisher for advantagescope swerve visualization
    StructArrayPublisher<SwerveModuleState> swerveStatePublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("/RBR/SwerveStates", SwerveModuleState.struct).publish();;
    // networktables publisher for advantagescope 2d pose visualization
    StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("/RBR/PoseEstimated", Pose2d.struct).publish();

    public Drivetrain() {
        // Configure AutoBuilder last
        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants(Constants.PathPlanner.translation_P, Constants.PathPlanner.translation_I, Constants.PathPlanner.translation_D), // Translation PID constants
                        new PIDConstants(Constants.PathPlanner.rotation_P, Constants.PathPlanner.rotation_I, Constants.PathPlanner.rotation_D), // Rotation PID constants
                        Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND, // Max module speed, in m/s
                        Constants.Kinematics.DRIVETRAIN_BASE_RADIUS, // Drive base radius in meters. Distance from robot center to
                                                          // furthest module.
                        new ReplanningConfig()),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );

        // Initialize all modules
        backRightModule = new SwerveModule(
            Constants.CanID.SWERVE_MODULE_BACK_RIGHT_DRIVE_MOTOR,
            Constants.CanID.SWERVE_MODULE_BACK_RIGHT_STEER_MOTOR,
            Constants.CanID.SWERVE_MODULE_BACK_RIGHT_STEER_ENCODER);

        backLeftModule = new SwerveModule(
            Constants.CanID.SWERVE_MODULE_BACK_LEFT_DRIVE_MOTOR,
            Constants.CanID.SWERVE_MODULE_BACK_LEFT_STEER_MOTOR,
            Constants.CanID.SWERVE_MODULE_BACK_LEFT_STEER_ENCODER);

        frontRightModule = new SwerveModule(
            Constants.CanID.SWERVE_MODULE_FRONT_RIGHT_DRIVE_MOTOR,
            Constants.CanID.SWERVE_MODULE_FRONT_RIGHT_STEER_MOTOR,
            Constants.CanID.SWERVE_MODULE_FRONT_RIGHT_STEER_ENCODER);

        frontLeftModule = new SwerveModule(
            Constants.CanID.SWERVE_MODULE_FRONT_LEFT_DRIVE_MOTOR,
            Constants.CanID.SWERVE_MODULE_FRONT_LEFT_STEER_MOTOR,
            Constants.CanID.SWERVE_MODULE_FRONT_LEFT_STEER_ENCODER);

        // Zero all relative encoders
        frontLeftModule.resetEncoders();
        frontRightModule.resetEncoders();
        backLeftModule.resetEncoders();
        backRightModule.resetEncoders();

        // Initialize and zero gyro
        navx = new AHRS(SPI.Port.kMXP);

        // Create the poseEstimator with vectors to weight our vision measurements
        //See this post on how to tune the std deviation values: https://www.chiefdelphi.com/t/how-do-i-understand-standard-deviation-in-the-poseestimator-class/411492/10
        poseEstimator = new SwerveDrivePoseEstimator(
                Constants.Kinematics.SWERVE_KINEMATICS,
                getGyroscopeRotation(), getSwerveModulePositions(), new Pose2d(),
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
        
        zeroGyroscope();

        theMove = "default";
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the
     * robot is currently facing to the 'forwards' direction.
     */
    public void zeroGyroscope() {
        poseEstimator.resetPosition(getGyroscopeRotation(), getSwerveModulePositions(), new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), new Rotation2d(0)));
    }

    /**
     * Toggles whether or not the wheels are locked. If they are, the wheels are
     * crossed into an X pattern and any other drive input has no effect.
     */
    public void toggleWheelsLocked() {
        wheelsLocked = !wheelsLocked;
    }

    public double getGyroscopeAngle() {
        return -navx.getYaw(); // -180 to 180, 0 degres is forward, ccw is +
    }

    // Returns the measurment of the gyroscope yaw. Used for field-relative drive
    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(getGyroscopeAngle());
    }

    /** @return Pitch in degrees, -180 to 180 */
    public double getPitch() {
        return navx.getPitch();
    }

    /** @return Roll in degrees, -180 to 180 */
    public double getRoll() {
        return navx.getRoll();
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
                frontLeftModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(),
                backRightModule.getPosition()
        };
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroscopeRotation(), getSwerveModulePositions(), pose);
    }

    public Rotation2d getPoseRotation() {
        return poseEstimator.getEstimatedPosition().getRotation();
    }

    public void updateOdometry() {
        // Update odometry from swerve states
        poseEstimator.update(
                getGyroscopeRotation(),
                getSwerveModulePositions());

        if (Constants.Vision.VISION_ENABLED) {
            List<EstimatedRobotPose> visionPoseEstimates = visionSystem.getRobotPoseEstimation();
            for (EstimatedRobotPose visionPoseEstimate : visionPoseEstimates) {
                //TODO: define this method / test and/or use a constant here for the standard deviation
                // poseEstimator.setVisionMeasurementStdDevs(visionSystem.getVisionMeasurementStandardDeviation(visionPoseEstimate));
                poseEstimator.addVisionMeasurement(visionPoseEstimate.estimatedPose.toPose2d(), visionPoseEstimate.timestampSeconds);
            }
        }
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return chassisSpeeds;
    }

    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND);
        frontLeftModule.setState(states[0]);
        frontRightModule.setState(states[1]);
        backLeftModule.setState(states[2]);
        backRightModule.setState(states[3]);
    }

    /**
     * Decide where the center of rotation is going to be based on function call (we
     * got the moves)
     **/
    public void setMoves(String theMove) {
        this.theMove = theMove;
    }

    /**
     * Used to drive the robot with the provided ChassisSpeed object.
     * 
     * @param chassisSpeeds The translational and rotational velocities at which to
     *                      drive the robot.
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    /**
     * This method is run every 20 ms.
     * <p>
     * This method is used to command the individual module states based off the
     * ChassisSpeeds object
     */
    @Override
    public void periodic() {
        handleMoves();
        handleLocked();
        updateOdometry();
        updateTelemetry();
        //TODO: MJR - consider adding collision detection, eg. https://gist.githubusercontent.com/kauailabs/8c152fa14937b9cdf137/raw/900c99b23a1940e121ed1ae1abd589eb4050b5c1/CollisionDetection.java
    }

    private void handleMoves() {
        switch (theMove) {
            case "FL":
                states = Constants.Kinematics.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds, 
                    new Translation2d(Constants.Kinematics.DRIVETRAIN_TRACKWIDTH_METERS / 2., Constants.Kinematics.DRIVETRAIN_WHEELBASE_METERS / 2.)
                );
                break;
            case "FR":
                states = Constants.Kinematics.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds,
                    new Translation2d(Constants.Kinematics.DRIVETRAIN_TRACKWIDTH_METERS / 2., -Constants.Kinematics.DRIVETRAIN_WHEELBASE_METERS / 2.)
                );
                break;
            case "default":
                states = Constants.Kinematics.SWERVE_KINEMATICS.toSwerveModuleStates(chassisSpeeds);
                break;
        }
    }

    private void handleLocked() {
        if (!wheelsLocked) {
            // If we are not in wheel's locked mode, set the states normally
            frontLeftModule.setState(states[0]);
            frontRightModule.setState(states[1]);
            backLeftModule.setState(states[2]);
            backRightModule.setState(states[3]);
        } else {
            // If we are in wheel's locked mode, set the drive velocity to 0 so there is no
            // movment, and command the steer angle to either plus or minus 45 degrees to
            // form an X pattern.
            frontLeftModule.lockModule(45);
            frontRightModule.lockModule(-45);
            backLeftModule.lockModule(-45);
            backRightModule.lockModule(45);
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Kinematics.MAX_VELOCITY_METERS_PER_SECOND);
    }

    private void updateTelemetry() {
        // Publish gyro angle to shuffleboard
        gyroEntry.setDouble(poseEstimator.getEstimatedPosition().getRotation().getDegrees());

        if (poseEstimator.getEstimatedPosition().getRotation().getDegrees() == 0.0) {
            gyroWarningEntry.setBoolean(true);
        } else {
            gyroWarningEntry.setBoolean(false);
        }

        // Advantage scope things
        posePublisher.set(poseEstimator.getEstimatedPosition());

        swerveStatePublisher.set(new SwerveModuleState[] {
                states[0],
                states[1],
                states[2],
                states[3]
        });
    }

    public void setVisionSystem(VisionSystem visionSystem) {
        this.visionSystem = visionSystem;
    }
}
