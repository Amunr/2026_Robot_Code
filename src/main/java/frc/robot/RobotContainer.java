// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.climber;
import frc.robot.subsystems.intake;
import swervelib.SwerveInputStream;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
    DriveTrain driveTrain = new DriveTrain(new File(Filesystem.getDeployDirectory(), "swerve"));
    private climber m_climberSubsystem = new climber();
    private intake m_intakeSubsystem = new intake();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final CommandXboxController driverXbox =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public final CommandXboxController operatorXboxController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  //Swerve
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(driveTrain.getSwerveDrive(),
      () -> driverXbox.getLeftY(),
      () -> driverXbox.getLeftX())
      .withControllerRotationAxis(driverXbox::getRightX)
      .deadband(OperatorConstants.deadband)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);


  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative
   * input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
      driverXbox::getRightY)
      .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative
   * input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
      .allianceRelativeControl(false);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
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
    Command driveFieldOrientedDirectAngle      = driveTrain.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = driveTrain.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = driveTrain.driveFieldOriented(driveRobotOriented);
    driveTrain.setDefaultCommand(driveFieldOrientedAnglularVelocity);

    Command climbl1 = new SequentialCommandGroup(new InstantCommand(m_climberSubsystem::setL1Over),
    new InstantCommand(m_climberSubsystem::moveClimber),
    new WaitCommand(4),
    new InstantCommand(m_climberSubsystem::setL1),
    new InstantCommand(m_climberSubsystem::moveClimber),
    new WaitCommand(2)
    );
    /* 
     new JoystickButton(driverXbox.getHID(), XboxController.Button.kRightBumper.value)
    .onTrue(new InstantCommand(m_climberSubsystem::movePOS));
    new JoystickButton(driverXbox.getHID(), XboxController.Button.kLeftBumper.value)
    .onTrue(climbl1);*/
    new JoystickButton(driverXbox.getHID(), XboxController.Button.kB.value)
    .onTrue(new InstantCommand(m_climberSubsystem::climbUp));
    new JoystickButton(driverXbox.getHID(), XboxController.Button.kX.value)
    .onTrue(new InstantCommand(m_climberSubsystem::climbDown));

    Command intakeForward = new SequentialCommandGroup(
    new InstantCommand(m_intakeSubsystem::intakeFuel),
    new InstantCommand(m_intakeSubsystem::spinIntake)
    );
    new JoystickButton(driverXbox.getHID(), XboxController.Button.kA.value)
    .onTrue(new InstantCommand(m_intakeSubsystem::movePOS));
    new Trigger(() -> driverXbox.getRightTriggerAxis() > 0.3).whileTrue(new InstantCommand(m_intakeSubsystem::intakeFuel))
    .onFalse(new InstantCommand(m_intakeSubsystem::stopIntake));
    new Trigger(() -> driverXbox.getLeftTriggerAxis() > 0.3).whileTrue(new InstantCommand(m_intakeSubsystem::extakeFuel))
    .onFalse(new InstantCommand(m_intakeSubsystem::stopExtake));
    /* 
    new Trigger(new JoystickButton(driverXbox.getHID(), XboxController.Button.kLeftBumper.value)).whileTrue(new InstantCommand(m_intakeSubsystem::foldFor))
    .onFalse(new InstantCommand(m_intakeSubsystem::stopFoldFor));
    new Trigger(new JoystickButton(driverXbox.getHID(), XboxController.Button.kRightBumper.value)).whileTrue(new InstantCommand(m_intakeSubsystem::FoldRev))
    .onFalse(new InstantCommand(m_intakeSubsystem::stopFoldRev));
    */

    new Trigger(new JoystickButton(driverXbox.getHID(), XboxController.Button.kLeftBumper.value)).whileTrue(new InstantCommand(m_climberSubsystem::spinForward))
    .onFalse(new InstantCommand(m_climberSubsystem::stopClimb));
    new Trigger(new JoystickButton(driverXbox.getHID(), XboxController.Button.kRightBumper.value)).whileTrue(new InstantCommand(m_climberSubsystem::spinReverse))
    .onFalse(new InstantCommand(m_climberSubsystem::stopClimb));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }
  public void setMotorBrake(boolean brake) {
    driveTrain.setMotorBrake(brake);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
