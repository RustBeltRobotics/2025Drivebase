// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriverStation;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionSystem;
import frc.robot.util.Utilities;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // For limiting maximum speed (1.0 = 100% - full speed)
  private static final double MAX_SPEED_FACTOR = 1.0;

  // TODO: ** utilize design ideas in this post:  https://www.chiefdelphi.com/t/command-based-best-practices-for-2025-community-feedback/465602/143
  // For examples on structuring Subsystems, triggers and commands, see: 
  // https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/rapidreactcommandbot

  private final XboxController driverController = new XboxController(DriverStation.CONTROLLER_PORT_DRIVER);
  private final XboxController operatorController = new XboxController(DriverStation.CONTROLLER_PORT_OPERATOR);
  private final Drivetrain drivetrain = new Drivetrain();
  private final VisionSystem visionSystem;

  private final SendableChooser<Command> autoChooser;
  private final SendableChooser<Integer> startingPosisitonChooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();

    if (Constants.Vision.VISION_ENABLED) {
      visionSystem = new VisionSystem();
      drivetrain.setVisionSystem(visionSystem);
    }

    setDefaultCommands();
    // Configure the trigger bindings
    configureBindings();

    //TODO: register NamedCommands for pathplanner

    configureAutos();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // TODO: MJR
    // Pressing A button zeros the gyroscope
    new Trigger(driverController::getAButton).onTrue(new InstantCommand(() -> drivetrain.zeroGyroscope()));

    //These are triggers for the DPad directions
    // POVButton driverDPadUpButton = new POVButton(driverController, 0);
    // POVButton driverDPadDownButton = new POVButton(driverController, 180);
    // POVButton driverDPadLeftButton = new POVButton(driverController, 270);
    // POVButton driverDPadRightButton = new POVButton(driverController, 90);
    // POVButton operatorDPadUpButton = new POVButton(operatorController, 0);
    // POVButton operatorDPadDownButton = new POVButton(operatorController, 180);
    // POVButton operatorDPadLeftButton = new POVButton(operatorController, 270);
    // POVButton operatorDPadRightButton = new POVButton(operatorController, 90);
  }

  private void setDefaultCommands() {
    DoubleSupplier translationXSupplier = () -> -Utilities.modifyAxisGeneric(driverController.getLeftY(), 1.0, 0.05) * Constants.Kinematics.MAX_SWERVE_MODULE_VELOCITY_METERS_PER_SECOND * MAX_SPEED_FACTOR;
    DoubleSupplier translationYSupplier = () -> -Utilities.modifyAxisGeneric(driverController.getLeftX(), 1.0, 0.05) * Constants.Kinematics.MAX_SWERVE_MODULE_VELOCITY_METERS_PER_SECOND * MAX_SPEED_FACTOR;
    DoubleSupplier rotationSupplier = () -> -Utilities.modifyAxisGeneric(driverController.getRightX(), 1.0, 0.05) * Constants.Kinematics.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * MAX_SPEED_FACTOR;
    
    drivetrain.setDefaultCommand(new FieldOrientedDriveCommand(drivetrain, translationXSupplier, translationYSupplier, rotationSupplier));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void configureAutos() {
    Constants.Shuffleboard.COMPETITION_TAB.add("auto machine", autoChooser).withPosition(0, 0).withSize(2, 1);

    startingPosisitonChooser.addOption("1", 1);
    startingPosisitonChooser.addOption("2", 2);
    startingPosisitonChooser.addOption("3", 3);
    Constants.Shuffleboard.COMPETITION_TAB.add("where am I?", startingPosisitonChooser).withPosition(2, 0);
  }

  //MJR: TODO: If we end up needing this, call it like something like this:
    // new InstantCommand(() -> rumbleControllers(true)).andThen(new WaitCommand(1.0)).finallyDo(() -> rumbleControllers(false));
  public void rumbleControllers(boolean rumble) {
    double rumbleValue = rumble ? 0.5 : 0.0;
    
    driverController.setRumble(RumbleType.kLeftRumble, rumbleValue);
    driverController.setRumble(RumbleType.kRightRumble, rumbleValue);
    operatorController.setRumble(RumbleType.kLeftRumble, rumbleValue);
    operatorController.setRumble(RumbleType.kRightRumble, rumbleValue);
  }
}
