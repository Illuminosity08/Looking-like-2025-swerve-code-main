package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  // Subsystems
  private final Swerve swerve = new Swerve();
  private final Vision vision = new Vision();
  private final Elevator elevator = new Elevator();
  private final Intake intake = new Intake(11);
  private final Arm arm = new Arm(13);

  // Controls
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final Joystick operatorJoystick = new Joystick(1);

  public RobotContainer() {

    // =========================
    // Limelight 3A Camera Stream
    // =========================
    HttpCamera limelightCam = new HttpCamera(
        "Limelight3A",
        "http://172.29.0.1:5800/stream.mjpg",
        HttpCamera.HttpCameraKind.kMJPGStreamer
    );
    limelightCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    CameraServer.startAutomaticCapture(limelightCam);

    // =========================
    // Named commands / events
    // =========================
    new EventTrigger("run elevator").whileTrue(Commands.print("running elevator"));
    new EventTrigger("drop coral").and(new Trigger(() -> elevator.getHeight() > 0.5))
        .onTrue(Commands.print("drop coral"));

    NamedCommands.registerCommand("elevatorUp",
        new InstantCommand(() -> elevator.setElevatorPosition(1.0)));
    NamedCommands.registerCommand("stopRobot",
        new InstantCommand(() -> swerve.drive(new ChassisSpeeds(0, 0, 0), false)));

    // Auto chooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Bind buttons
    configureBindings();

    // Default drive = FIELD RELATIVE manual
    Command driveCommand = new RunCommand(() -> {
      swerve.driveTeleopFieldRelative(
          -m_driverController.getLeftY(),
          -m_driverController.getLeftX(),
          -m_driverController.getRightX()
      );
    }, swerve);

    swerve.setDefaultCommand(driveCommand);
  }

  private void configureBindings() {

    // =========================
    // Operator controls (your existing stuff)
    // =========================
    JoystickButton presetLow = new JoystickButton(operatorJoystick, 2);
    JoystickButton presetHigh = new JoystickButton(operatorJoystick, 3);
    JoystickButton resetElevator = new JoystickButton(operatorJoystick, 4);
    JoystickButton intakeButton = new JoystickButton(operatorJoystick, 5);
    JoystickButton ejectButton = new JoystickButton(operatorJoystick, 6);
    JoystickButton stopIntakeButton = new JoystickButton(operatorJoystick, 7);

    intakeButton.whileTrue(new RunCommand(intake::intake, intake));
    ejectButton.whileTrue(new RunCommand(intake::eject, intake));
    stopIntakeButton.onTrue(new InstantCommand(intake::stop, intake));

    presetLow.onTrue(new InstantCommand(() -> elevator.setElevatorPosition(0.3)));
    presetHigh.onTrue(new InstantCommand(() -> elevator.setElevatorPosition(0.2)));
    resetElevator.onTrue(new InstantCommand(() -> elevator.setElevatorPosition(0.0)));

    // Reset pose heading to alliance-facing
    m_driverController.start().onTrue(new InstantCommand(() -> {
      double deg = (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) ? 180 : 0;
      swerve.resetPose(new Pose2d(
          swerve.getPose().getTranslation(),
          new Rotation2d(Degrees.of(deg))
      ));
    }));

    elevator.setDefaultCommand(
        new RunCommand(() -> elevator.setElevatorSpeed(operatorJoystick.getRawAxis(5)), elevator));

    arm.setDefaultCommand(
        new RunCommand(() -> arm.setPower(operatorJoystick.getRawAxis(1) * 0.2), arm));

    // Reset rotation to face away from driver station
    m_driverController.leftBumper().onTrue(Commands.runOnce(() -> {
      var currentPose = swerve.getPose();
      Rotation2d newRotation = Rotation2d.fromDegrees(0); // blue
      if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
        newRotation = Rotation2d.fromDegrees(180);
      }
      swerve.resetPose(new Pose2d(currentPose.getTranslation(), newRotation));
    }));

    // =========================
    // TELEOP AUTO-ALIGN (A button) to APRILTAG 24
    // =========================
    m_driverController.a().whileTrue(
        new RunCommand(() -> {

          // Fallback manual drive (field-relative)
          Runnable fallbackDrive = () -> swerve.driveTeleopFieldRelative(
              -m_driverController.getLeftY(),
              -m_driverController.getLeftX(),
              -m_driverController.getRightX()
          );

          // Only align when we see Tag 24
          if (!vision.hasTarget() || vision.getTagId() != 23) {
            fallbackDrive.run();
            return;
          }

          // targetpose_robotspace: [x, y, z, roll, pitch, yaw]
          double[] pose = vision.getTargetPoseRobotSpace();
          double xMeters = pose[0];  // forward distance to tag (m)
          double yMeters = pose[1];  // left/right offset (m)
          double yawDeg  = pose[5];  // yaw to tag (deg)

          // Desired stop distance from the tag
          double desiredX = .60; // meters (tune)
          double errX = xMeters - desiredX;
          double errY = yMeters;
          double errYaw = yawDeg;

          // Gains (safe starting points)
          double kPX = 0.8;
          double kPY = 0.8;
          double kPYaw = 0.02;

          // Convert to joystick-like commands (-1..1) with clamps
          double xCmd = clamp(-kPX * errX, -0.6, 0.6);
          double yCmd = clamp(+kPY * errY, -0.6, 0.6);
          double rotCmd = clamp(-kPYaw * errYaw, -0.5, 0.5);

          // Deadbands to stop jitter
          if (Math.abs(errX) < 0.05) xCmd = 0;     // 5cm
          if (Math.abs(errY) < 0.05) yCmd = 0;     // 5cm
          if (Math.abs(errYaw) < 1.0) rotCmd = 0;  // 1deg

          // Drive field-relative
          swerve.driveTeleopFieldRelative(xCmd, yCmd, rotCmd);

        }, swerve)
    );
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("test");
  }

  private static double clamp(double val, double min, double max) {
    return Math.max(min, Math.min(max, val));
  }
}
