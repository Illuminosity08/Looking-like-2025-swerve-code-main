// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final SendableChooser<Command> autoChooser;

  // The robot's subsystems and commands are defined here...
  private final Swerve swerve = new Swerve();
private final Elevator elevator = new Elevator();
private final Intake intake = new Intake(5); // Change 5 to the actual CAN ID
private final Joystick operatorJoystick = new Joystick(1);

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    new EventTrigger("run elevator").whileTrue(Commands.print("running elevator"));
    new EventTrigger("drop coral").and(new Trigger(() -> elevator.getHeight() > 0.5)).onTrue(Commands.print("drop coral"));
    NamedCommands.registerCommand("elevatorUp", new InstantCommand(() -> elevator.setElevatorPosition(1.0)));
    NamedCommands.registerCommand("stopRobot", new InstantCommand(() -> swerve.drive(new ChassisSpeeds(0, 0, 0), false)));

        autoChooser = AutoBuilder.buildAutoChooser(); 
       SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
    Command driveCommand = swerve.driveCommand(() -> -m_driverController.getLeftY(),
    () -> -m_driverController.getLeftX(), () -> -m_driverController.getRightX());

    swerve.setDefaultCommand(driveCommand);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   * \[]
   */
  private void configureBindings() {
    JoystickButton presetLow = new JoystickButton(operatorJoystick, 2); // Button 2
    JoystickButton presetHigh = new JoystickButton(operatorJoystick, 3); // Button 3
    JoystickButton resetElevator = new JoystickButton(operatorJoystick, 4); // Button 4
    JoystickButton intakeButton = new JoystickButton(operatorJoystick, 5); // Button 5
    JoystickButton ejectButton = new JoystickButton(operatorJoystick, 6);  // Button 6
    JoystickButton stopIntakeButton = new JoystickButton(operatorJoystick, 7); // Button 7
    
    intakeButton.whileTrue(new InstantCommand(intake::intake, intake));
    ejectButton.whileTrue(new InstantCommand(intake::eject, intake));
    stopIntakeButton.onTrue(new InstantCommand(intake::stop, intake));    
    presetLow.onTrue(new InstantCommand(() -> elevator.setElevatorPosition(0.3))); // Move to 30 cm
    presetHigh.onTrue(new InstantCommand(() -> elevator.setElevatorPosition(1.2))); // Move to 120 cm
    resetElevator.onTrue(new InstantCommand(() -> elevator.setElevatorPosition(0.0))); // Moves to base position



    // Reset rotation to face away from driver statoin
    m_driverController.leftBumper().onTrue(Commands.runOnce(() -> {
      var currentPose = swerve.getPose();
      Rotation2d newRotation = Rotation2d.fromDegrees(0); // blue side
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
        newRotation = Rotation2d.fromDegrees(180);
      }
      swerve.resetPose(new Pose2d(currentPose.getTranslation(), newRotation));
    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    Command selectedAuto = autoChooser.getSelected();  // Get selected auto mode from SmartDashboard

    if (selectedAuto != null) {
        return selectedAuto;  // Run selected auto command
    } else {
        DriverStation.reportError("No auto mode selected!", false);
        return Commands.none();  // Failsafe to prevent crashes
    }
  }}


  
    
