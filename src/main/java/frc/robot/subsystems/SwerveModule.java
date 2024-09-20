package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Utilities;

import static frc.robot.Constants.*;

/**
 * Models an individual wheel swerve module: drive motor, steer motor, encoders (for reading distance traveled, turn angle, steer velocity, etc.)
 */
public class SwerveModule extends SubsystemBase {

    // Drive PID Constants
    public static final double DRIVE_P = 0.2;
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;
    public static final double DRIVE_F = 0.22;

    // Steer PID Constants
    public static final double STEER_P = 0.01;
    public static final double STEER_I = 0.0;
    public static final double STEER_D = 0.0;
    public static final double STEER_F = 0.0;

    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;

    private SparkPIDController drivePidController;
    private SparkPIDController steerPidController;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;

    private final CANcoder absoluteSteerEncoder;

    public SwerveModule(int driveID, int steerID, int encoderID) {
        //TODO: MJR this code is for SDS MK4 - we will be using MK4i, update accordingly (still L3 gear ratio)
        // Setup drive motor SparkMax
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();
        driveMotor.setIdleMode(IdleMode.kBrake);
        driveMotor.setInverted(true);
        driveMotor.setSmartCurrentLimit(Constants.CurrentLimit.SparkMax.SMART_DRIVE);
        driveMotor.setSecondaryCurrentLimit(Constants.CurrentLimit.SparkMax.SECONDARY_DRIVE);

        // Setup PID functionality for drive motors
        drivePidController = driveMotor.getPIDController();

        // Setup drive motor relative encoder
        driveEncoder = driveMotor.getEncoder();
        driveEncoder.setPositionConversionFactor(Constants.Kinematics.DRIVE_POSITION_CONVERSION);
        driveEncoder.setVelocityConversionFactor(Constants.Kinematics.DRIVE_VELOCITY_CONVERSION);

        // Setup steer motor SparkMax
        steerMotor = new CANSparkMax(steerID, MotorType.kBrushless);
        steerMotor.restoreFactoryDefaults();
        steerMotor.setIdleMode(IdleMode.kBrake);
        steerMotor.setInverted(false);
        steerMotor.setSmartCurrentLimit(Constants.CurrentLimit.SparkMax.SMART_STEER);
        steerMotor.setSecondaryCurrentLimit(Constants.CurrentLimit.SparkMax.SECONDARY_STEER);

        // Setup PID functionality for steer motors
        steerPidController = steerMotor.getPIDController();

        // Setup steer motor relative encoder
        steerEncoder = steerMotor.getEncoder();
        steerEncoder.setPositionConversionFactor(Constants.Kinematics.STEER_POSITION_CONVERSION);
        steerEncoder.setVelocityConversionFactor(Constants.Kinematics.STEER_VELOCITY_CONVERSION);

        // Setup steer motor absolute encoder
        absoluteSteerEncoder = new CANcoder(encoderID);

        // Zero encoders to ensure steer relative matches absolute
        resetEncoders();
        //set the things
        updatePIDs();
    }
    
    public void updatePIDs() {
        // set PID coefficients (drive)
        drivePidController.setP(DRIVE_P);
        drivePidController.setI(DRIVE_I);
        drivePidController.setD(DRIVE_D);
        drivePidController.setFF(DRIVE_F);
        drivePidController.setIZone(0.0);
        //drivePidController.setOutputRange(kMinOutput.getDouble(-1.), kMaxOutput.getDouble(1.));
        drivePidController.setPositionPIDWrappingEnabled(false);

        // set PID coefficients (steer)
        steerPidController.setP(STEER_P);
        steerPidController.setI(STEER_I);
        steerPidController.setD(STEER_D);
        // Does it make sense to use the same value here as for drive? - i dont know what this value does
        // Basically, this limits the range of error values that are included in your
        // integral. If you have a large error, its ignored, because large errors can
        // lead to integral windup, which is reeeally bad. as long as our kI is 0, this
        // IZone value is irrelevant, but if we ever want to incorporate an I gain,
        // we'll definitely want to keep an eye on this one.
        steerPidController.setIZone(0.0);
        steerPidController.setFF(STEER_F);
        steerPidController.setOutputRange(-1.0, 1.0);
        steerPidController.setPositionPIDWrappingEnabled(true);
        steerPidController.setPositionPIDWrappingMaxInput(360.0);
        steerPidController.setPositionPIDWrappingMinInput(0.0);
    }

    /** @return Drive position, meters, -inf to +inf */
    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    /** @return Steer position, degrees, -inf to +inf */
    public double getSteerPosition() {
        return steerEncoder.getPosition();
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
        return absoluteSteerEncoder.getAbsolutePosition().getValueAsDouble() * 360.;
    }

    /** @return Drive encoder (meters) and steer encoder (Rotation2d) positions */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), new Rotation2d(Math.toRadians(getSteerPosition())));
    }

    /**
     * Resets the drive relative encoder to 0 and steer relative encoder to match
     * absolute encoder
     */
    public void resetEncoders() {
        driveEncoder.setPosition(0.0);
        steerEncoder.setPosition(getAbsolutePosition());
    }

    /**
     * @return The module state (velocity, m/s, and steer angle, Rotation2d)
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(Math.toRadians(getSteerPosition())));
    }

    /**
     * Set's the speed and angle of an idividual module.
     * 
     * @param state the desired state (velocity, m/s, and steer angle, Rotation2d)
     */
    public void setState(SwerveModuleState state) {
        // If input is minimal, ignore input to avoid reseting steer angle to 0 degrees
        if (Math.abs(state.speedMetersPerSecond) < 0.01) {
            stopModule();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        drivePidController.setReference(state.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
        setSteerAngle(state.angle.getDegrees());
    }

    /**
     * Locks the wheel at the provided angle
     * 
     * @param angle degrees
     */
    public void lockModule(int angle) {
        steerPidController.setReference(angle, CANSparkMax.ControlType.kPosition);
    }

    /** Set's the voltage to both motors to 0 */
    public void stopModule() {
        driveMotor.set(0.);
        steerMotor.set(0.);
    }

    public void setSteerAngle(double targetAngleInDegrees) {
        double currentSparkAngle = getSteerPosition();
        double sparkRelativeTargetAngle = Utilities.reboundValue(targetAngleInDegrees, currentSparkAngle);
        steerPidController.setReference(sparkRelativeTargetAngle, ControlType.kPosition);
    }
}