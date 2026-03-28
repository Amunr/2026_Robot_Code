// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.climber;
import frc.robot.subsystems.intake;
import swervelib.SwerveInputStream;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
        NamedCommands.registerCommand("intake", intakeAuto);
    configureBindings();
      UsbCamera camera = CameraServer.startAutomaticCapture();
        ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
        driverTab.add("Camera", camera);
  }

  Command intakeAuto = new SequentialCommandGroup(new InstantCommand(m_intakeSubsystem::intakeFoldOut),new InstantCommand(m_intakeSubsystem::spinIntake), new WaitCommand(7), new InstantCommand(m_intakeSubsystem::stopIntake));
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


    /* 
     new JoystickButton(driverXbox.getHID(), XboxController.Button.kRightBumper.value)
    .onTrue(new InstantCommand(m_climberSubsystem::movePOS));
    new JoystickButton(driverXbox.getHID(), XboxController.Button.kLeftBumper.value)
    .onTrue(climbl1);*/
    new JoystickButton(driverXbox.getHID(), XboxController.Button.kB.value)
    .onTrue(new SequentialCommandGroup(new InstantCommand(m_climberSubsystem::pushout),new WaitCommand(1), new InstantCommand(m_climberSubsystem::hooksOut), new InstantCommand(m_intakeSubsystem::intakeFoldIn)));
    new JoystickButton(driverXbox.getHID(), XboxController.Button.kX.value)
    .onTrue(new SequentialCommandGroup(new InstantCommand(m_climberSubsystem::flipback), new WaitCommand(3), new InstantCommand(m_climberSubsystem::flipback), new WaitCommand(3), new InstantCommand(m_climberSubsystem::flipback), new WaitCommand(3), new InstantCommand(m_climberSubsystem::flipback), new WaitCommand(3), new InstantCommand(m_climberSubsystem::flipback)));

    new Trigger(() -> driverXbox.getRightTriggerAxis() > 0.3).whileTrue(new InstantCommand(m_intakeSubsystem::spinIntake))
    .onFalse(new InstantCommand(m_intakeSubsystem::stopIntake));
    new Trigger(() -> driverXbox.getLeftTriggerAxis() > 0.3).whileTrue(new InstantCommand(m_intakeSubsystem::reverseINtake))
    .onFalse(new InstantCommand(m_intakeSubsystem::stopIntake));

    new JoystickButton(driverXbox.getHID(), XboxController.Button.kY.value).onTrue(new InstantCommand(m_climberSubsystem::resetClimbEncoder));
    
    new Trigger(new JoystickButton(driverXbox.getHID(), XboxController.Button.kLeftBumper.value)).onTrue(new InstantCommand(m_intakeSubsystem::intakeFoldOut));
    new Trigger(new JoystickButton(driverXbox.getHID(), XboxController.Button.kRightBumper.value)).whileTrue(new InstantCommand(m_intakeSubsystem::intakeFoldIn));
  

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

    public Command getAutonomousCommand(String pathName)
  {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
