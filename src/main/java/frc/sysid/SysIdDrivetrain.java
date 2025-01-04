package frc.sysid;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

/**
 * Drive subsystem for SysId characterization of swerve drive motors.
 */
public class SysIdDrivetrain extends SubsystemBase {
    
    private final CANSparkMax frontLeftDriveMotor;
    private final RelativeEncoder frontLeftDriveEncoder;
    private final CANSparkMax frontRightDriveMotor;
    private final RelativeEncoder frontRightDriveEncoder;
    private final CANSparkMax backRightDriveMotor;
    private final RelativeEncoder backRightDriveEncoder;
    private final CANSparkMax backLeftDriveMotor;
    private final RelativeEncoder backLeftDriveEncoder;
    
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

    private final SysIdRoutine m_sysIdRoutine;

    public SysIdDrivetrain() {
        // Setup drive motor controller
        frontLeftDriveMotor = new CANSparkMax(Constants.CanID.SWERVE_MODULE_FRONT_LEFT_DRIVE_MOTOR, MotorType.kBrushless);
        frontLeftDriveMotor.restoreFactoryDefaults();
        frontLeftDriveMotor.setIdleMode(IdleMode.kBrake);
        frontLeftDriveMotor.setInverted(Constants.Kinematics.DRIVE_MOTOR_INVERTED);
        frontLeftDriveMotor.setSmartCurrentLimit(Constants.CurrentLimit.SparkMax.SMART_DRIVE);
        frontLeftDriveMotor.setSecondaryCurrentLimit(Constants.CurrentLimit.SparkMax.SECONDARY_DRIVE);

        // Setup drive motor relative encoder
        frontLeftDriveEncoder = frontLeftDriveMotor.getEncoder();
        frontLeftDriveEncoder.setPositionConversionFactor(Constants.Kinematics.DRIVE_POSITION_CONVERSION);
        frontLeftDriveEncoder.setVelocityConversionFactor(Constants.Kinematics.DRIVE_VELOCITY_CONVERSION);
        frontLeftDriveEncoder.setPosition(0.0);

        frontRightDriveMotor = new CANSparkMax(Constants.CanID.SWERVE_MODULE_FRONT_RIGHT_DRIVE_MOTOR, MotorType.kBrushless);
        frontRightDriveMotor.restoreFactoryDefaults();
        frontRightDriveMotor.setIdleMode(IdleMode.kBrake);
        frontRightDriveMotor.setInverted(Constants.Kinematics.DRIVE_MOTOR_INVERTED);
        frontRightDriveMotor.setSmartCurrentLimit(Constants.CurrentLimit.SparkMax.SMART_DRIVE);
        frontRightDriveMotor.setSecondaryCurrentLimit(Constants.CurrentLimit.SparkMax.SECONDARY_DRIVE);

        frontRightDriveEncoder = frontRightDriveMotor.getEncoder();
        frontRightDriveEncoder.setPositionConversionFactor(Constants.Kinematics.DRIVE_POSITION_CONVERSION);
        frontRightDriveEncoder.setVelocityConversionFactor(Constants.Kinematics.DRIVE_VELOCITY_CONVERSION);
        frontRightDriveEncoder.setPosition(0.0);

        backRightDriveMotor = new CANSparkMax(Constants.CanID.SWERVE_MODULE_BACK_RIGHT_DRIVE_MOTOR, MotorType.kBrushless);
        backRightDriveMotor.restoreFactoryDefaults();
        backRightDriveMotor.setIdleMode(IdleMode.kBrake);
        backRightDriveMotor.setInverted(Constants.Kinematics.DRIVE_MOTOR_INVERTED);
        backRightDriveMotor.setSmartCurrentLimit(Constants.CurrentLimit.SparkMax.SMART_DRIVE);
        backRightDriveMotor.setSecondaryCurrentLimit(Constants.CurrentLimit.SparkMax.SECONDARY_DRIVE);

        backRightDriveEncoder = backRightDriveMotor.getEncoder();
        backRightDriveEncoder.setPositionConversionFactor(Constants.Kinematics.DRIVE_POSITION_CONVERSION);
        backRightDriveEncoder.setVelocityConversionFactor(Constants.Kinematics.DRIVE_VELOCITY_CONVERSION);
        backRightDriveEncoder.setPosition(0.0);

        backLeftDriveMotor = new CANSparkMax(Constants.CanID.SWERVE_MODULE_BACK_LEFT_DRIVE_MOTOR, MotorType.kBrushless);
        backLeftDriveMotor.restoreFactoryDefaults();
        backLeftDriveMotor.setIdleMode(IdleMode.kBrake);
        backLeftDriveMotor.setInverted(Constants.Kinematics.DRIVE_MOTOR_INVERTED);
        backLeftDriveMotor.setSmartCurrentLimit(Constants.CurrentLimit.SparkMax.SMART_DRIVE);
        backLeftDriveMotor.setSecondaryCurrentLimit(Constants.CurrentLimit.SparkMax.SECONDARY_DRIVE);

        backLeftDriveEncoder = backLeftDriveMotor.getEncoder();
        backLeftDriveEncoder.setPositionConversionFactor(Constants.Kinematics.DRIVE_POSITION_CONVERSION);
        backLeftDriveEncoder.setVelocityConversionFactor(Constants.Kinematics.DRIVE_VELOCITY_CONVERSION);
        backLeftDriveEncoder.setPosition(0.0);

        //TODO: MJR figure out why we get this error when loadfing data in sysid
        // https://www.chiefdelphi.com/t/sysid-routine-not-properly-recording-motor-speed/455172

        // Create a new SysId routine for characterizing the drive.
        m_sysIdRoutine = new SysIdRoutine(
            // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                // Tell SysId how to plumb the driving voltage to the motors.
                (Measure<Voltage> volts) -> {
                    frontLeftDriveMotor.setVoltage(volts.in(Volts));
                    frontRightDriveMotor.setVoltage(volts.in(Volts));
                    backRightDriveMotor.setVoltage(volts.in(Volts));
                    backLeftDriveMotor.setVoltage(volts.in(Volts));
                },
                // Tell SysId how to record a frame of data for each motor on the mechanism being
                // characterized.
                log -> {
                    // Record a frame for the motors.
                    log.motor("drive-front-left")
                        // .voltage(m_appliedVoltage.mut_replace(frontLeftDriveMotor.get() * RobotController.getBatteryVoltage(), Volts))
                        .voltage(m_appliedVoltage.mut_replace(frontLeftDriveMotor.getAppliedOutput() * frontLeftDriveMotor.getBusVoltage(), Volts))
                        .linearPosition(m_distance.mut_replace(frontLeftDriveEncoder.getPosition(), Meters))
                        .linearVelocity(m_velocity.mut_replace(frontLeftDriveEncoder.getVelocity(), MetersPerSecond));
                    log.motor("drive-front-right")
                        // .voltage(m_appliedVoltage.mut_replace(frontRightDriveMotor.get() * RobotController.getBatteryVoltage(), Volts))
                        .voltage(m_appliedVoltage.mut_replace(frontLeftDriveMotor.getAppliedOutput() * frontLeftDriveMotor.getBusVoltage(), Volts))
                        .linearPosition(m_distance.mut_replace(frontRightDriveEncoder.getPosition(), Meters))
                        .linearVelocity(m_velocity.mut_replace(frontRightDriveEncoder.getVelocity(), MetersPerSecond));
                    log.motor("drive-back-right")
                        // .voltage(m_appliedVoltage.mut_replace(backRightDriveMotor.get() * RobotController.getBatteryVoltage(), Volts))
                        .voltage(m_appliedVoltage.mut_replace(frontLeftDriveMotor.getAppliedOutput() * frontLeftDriveMotor.getBusVoltage(), Volts))
                        .linearPosition(m_distance.mut_replace(backRightDriveEncoder.getPosition(), Meters))
                        .linearVelocity(m_velocity.mut_replace(backRightDriveEncoder.getVelocity(), MetersPerSecond));
                    log.motor("drive-back-left")
                        // .voltage(m_appliedVoltage.mut_replace(backLeftDriveMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .voltage(m_appliedVoltage.mut_replace(frontLeftDriveMotor.getAppliedOutput() * frontLeftDriveMotor.getBusVoltage(), Volts))
                        .linearPosition(m_distance.mut_replace(backLeftDriveEncoder.getPosition(), Meters))
                        .linearVelocity(m_velocity.mut_replace(backLeftDriveEncoder.getVelocity(), MetersPerSecond));
                },
                // Tell SysId to make generated commands require this subsystem, suffix test state in
                // WPILog with this subsystem's name ("drive")
                this)
        );
    }

      /**
     * Returns a command that will execute a quasistatic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    /**
     * Returns a command that will execute a dynamic test in the given direction.
     *
     * @param direction The direction (forward or reverse) to run the test in
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}
