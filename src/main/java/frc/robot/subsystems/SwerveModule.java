package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BasicUnits;
import frc.robot.util.Utilities;

/**
 * Models an individual wheel swerve module: drive motor, steer motor, encoders (for reading distance traveled, turn angle, steer velocity, etc.)
 */
public class SwerveModule extends SubsystemBase {

    // Drive PID Constants
    public static final double DRIVE_FEEDFORWARD_KV = 2.3489;  //From SysId, confirmed with recalc https://www.reca.lc/drive
    // public static final double DRIVE_P = 0.2;  //From Eclipse
    public static final double DRIVE_P = 0.00044742;  //From SysId
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;
    // public static final double DRIVE_FF = 0.22;  //From Eclipse
    public static final double DRIVE_FF = 1.0 / DRIVE_FEEDFORWARD_KV;

    // Steer PID Constants
    // For how to tune these values, see: https://www.chiefdelphi.com/t/official-sds-mk3-mk4-code/397109/17
    //TODO: test with these values as well
    //  https://github.com/SwerveDriveSpecialties/Do-not-use-swerve-lib-2022-unmaintained/blob/develop/src/main/java/com/swervedrivespecialties/swervelib/Mk4iSwerveModuleHelper.java#L40
    //  https://yagsl.gitbook.io/yagsl/configuring-yagsl/how-to-tune-pidf#starting-points
    public static final double STEER_P = 0.01;
    public static final double STEER_I = 0.0;
    public static final double STEER_D = 0.0;
    public static final double STEER_FF = 0.0;

    private final int driveCanID;
    private final int steerCanID;

    private final SparkMax driveMotor;
    private final SparkMax steerMotor;

    private SparkClosedLoopController drivePidController;
    private SparkClosedLoopController steerPidController;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;

    private final CANcoder absoluteSteerEncoder;

    public SwerveModule(int driveID, int steerID, int encoderID) {
        this.driveCanID = driveID;
        this.steerCanID = steerID;
        // Setup drive motor SparkMax
        SparkMaxConfig driveMotorSparkMaxConfig = getDriveSparkConfig();
        driveMotor = new SparkMax(driveID, MotorType.kBrushless);
        REVLibError driveConfigResult = driveMotor.configure(driveMotorSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        Utilities.verifySparkMaxStatus(driveConfigResult, driveID, "Swerve Drive Motor", "configure");

        // Setup PID functionality for drive motors
        drivePidController = driveMotor.getClosedLoopController();

        // Setup drive motor relative encoder
        driveEncoder = driveMotor.getEncoder();

        // Setup steer motor SparkMax
        SparkMaxConfig steerMotorSparkMaxConfig = getSteerSparkConfig();
        steerMotor = new SparkMax(steerID, MotorType.kBrushless);
        REVLibError steerConfigResult = steerMotor.configure(steerMotorSparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        Utilities.verifySparkMaxStatus(steerConfigResult, steerID, "Swerve Steer Motor", "configure");

        // Setup PID functionality for steer motors
        steerPidController = steerMotor.getClosedLoopController();
        // Setup steer motor relative encoder
        steerEncoder = steerMotor.getEncoder();

        // Setup steer motor absolute encoder
        absoluteSteerEncoder = new CANcoder(encoderID);

        resetEncoders(); // Zero encoders to ensure steer relative matches absolute
    }

    private SparkMaxConfig getDriveSparkConfig() {
        SparkMaxConfig driveMotorSparkMaxConfig = new SparkMaxConfig();
        driveMotorSparkMaxConfig.inverted(Constants.Kinematics.DRIVE_MOTOR_INVERTED).idleMode(IdleMode.kBrake);
        driveMotorSparkMaxConfig.smartCurrentLimit(Constants.CurrentLimit.SparkMax.SMART_DRIVE).secondaryCurrentLimit(Constants.CurrentLimit.SparkMax.SECONDARY_DRIVE);
        driveMotorSparkMaxConfig.encoder.positionConversionFactor(Constants.Kinematics.DRIVE_POSITION_CONVERSION).velocityConversionFactor(Constants.Kinematics.DRIVE_VELOCITY_CONVERSION);
        driveMotorSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_FF).iZone(0.0).positionWrappingEnabled(false);

        return driveMotorSparkMaxConfig;
    }

    private SparkMaxConfig getSteerSparkConfig() {
        SparkMaxConfig steerMotorSparkMaxConfig = new SparkMaxConfig();
        steerMotorSparkMaxConfig.inverted(Constants.Kinematics.STEER_MOTOR_INVERTED).idleMode(IdleMode.kBrake);
        steerMotorSparkMaxConfig.smartCurrentLimit(Constants.CurrentLimit.SparkMax.SMART_STEER).secondaryCurrentLimit(Constants.CurrentLimit.SparkMax.SECONDARY_STEER);
        steerMotorSparkMaxConfig.encoder.positionConversionFactor(Constants.Kinematics.STEER_POSITION_CONVERSION).velocityConversionFactor(Constants.Kinematics.STEER_VELOCITY_CONVERSION);
        // Enable PID wrap around for the turning motor. This will allow the PID
        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
        // to 10 degrees will go through 0 rather than the other direction which is a
        // longer route.
        steerMotorSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pidf(STEER_P, STEER_I, STEER_D, STEER_FF).iZone(0.0).outputRange(-1.0, 1.0)
            .positionWrappingEnabled(true).positionWrappingMaxInput(360.0).positionWrappingMinInput(0.0);

        return steerMotorSparkMaxConfig;
    }

    /** @return Drive position, meters, -inf to +inf */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /** @return Steer position, degrees, -inf to +inf */
    public double getSteerPosition() {
        return steerEncoder.getPosition();
    }

    /*
     * Get angle of steer motor in range -180 to 180
     */
    public double getSteerPositionConstrained() {
        double rawPosition = steerEncoder.getPosition();

        return rawPosition % 180.0;
    }

    /** @return Drive position, meters/second */
    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    /** @return Steer position, degrees/second */
    public double getSteerVelocity() {
        return steerEncoder.getVelocity();
    }

    /** @return Absolute steer position, degrees, -inf to +inf */
    public double getAbsolutePosition() {
        return absoluteSteerEncoder.getAbsolutePosition().getValueAsDouble() * BasicUnits.DEGREES_PER_REVOLUTION;
    }

    /** @return Position of this swerve module based on Drive encoder (meters) and steer encoder (Rotation2d in radians) positions */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(Math.toRadians(getSteerPosition())));
    }

    /**
     * Resets the drive relative encoder to 0 and steer relative encoder to match
     * absolute encoder
     */
    public void resetEncoders() {
        REVLibError driveResult = driveEncoder.setPosition(0.0);
        Utilities.verifySparkMaxStatus(driveResult, driveCanID, "Swerve Drive Motor", "resetEncoderPosition");
        REVLibError steerResult = steerEncoder.setPosition(getAbsolutePosition());
        Utilities.verifySparkMaxStatus(steerResult, steerCanID, "Swerve Steer Motor", "resetEncoderPosition");
    }

    /**
     * @return The module state (velocity, m/s, and steer angle, Rotation2d)
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getSteerPosition()));
    }

    /**
     * @return The module state (velocity, m/s, and steer angle, Rotation2d - constrained to -pi to pi radians)
     */
    public SwerveModuleState getStateRotationConstrained() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getSteerPositionConstrained()));
    }

    /**
     * Sets the speed and angle of this swerve module
     * 
     * @param state the desired state (velocity in m/s, and steer angle as Rotation2d)
     */
    public void setState(SwerveModuleState state) {
        // If input is minimal, ignore input to avoid reseting steer angle to 0 degrees
        if (Math.abs(state.speedMetersPerSecond) < 0.01) {
            stopModule();
            return;
        }
        
        Rotation2d currentAngle = Rotation2d.fromDegrees(getSteerPosition());
        // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#module-angle-optimization
        state.optimize(currentAngle);
        // https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#cosine-compensation
        state.cosineScale(currentAngle);
        REVLibError pidResult = drivePidController.setReference(state.speedMetersPerSecond, SparkMax.ControlType.kVelocity);
        Utilities.verifySparkMaxStatus(pidResult, driveCanID, "Swerve Drive Motor", "setPIDReference");
        setSteerAngle(state.angle.getDegrees());
    }

    /**
     * Locks the wheel at the provided angle
     * 
     * @param angle degrees
     */
    public void lockModule(int angle) {
        steerPidController.setReference(angle, SparkMax.ControlType.kPosition);
    }

    /** Set's the voltage to both motors to 0 */
    public void stopModule() {
        driveMotor.set(0.);
        steerMotor.set(0.);
    }

    public void setSteerAngle(double targetAngleInDegrees) {
        double currentSparkAngle = getSteerPosition();
        double sparkRelativeTargetAngle = Utilities.reboundValue(targetAngleInDegrees, currentSparkAngle);
        REVLibError pidResult = steerPidController.setReference(sparkRelativeTargetAngle, ControlType.kPosition);
        Utilities.verifySparkMaxStatus(pidResult, steerCanID, "Swerve Steer Motor", "setPIDReference");
    }

    public SparkClosedLoopController getDrivePidController() {
        return drivePidController;
    }

    public SparkClosedLoopController getSteerPidController() {
        return steerPidController;
    }

    public SparkMax getDriveMotor() {
        return driveMotor;
    }

    public SparkMax getSteerMotor() {
        return steerMotor;
    }

    
}