// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {

  // Max speed used by YAGSL (you had 4 ft/s here)
  double maximumSpeed = Units.feetToMeters(2);
  File directory = new File(Filesystem.getDeployDirectory(), "swerve");
  SwerveDrive swerveDrive;
  Field2d newField = new Field2d();

  public Swerve() {

    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose,          // Robot pose supplier
        this::resetPose,        // Method to reset odometry
        this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> drive(speeds, false), // Drive with ROBOT RELATIVE speeds
        new PPHolonomicDriveController(
            new PIDConstants(0.10, 0.0, 0.0), // Translation PID
            new PIDConstants(5.0, 0.0, 0.0)   // Rotation PID
        ),
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this
    );

    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    swerveDrive.setHeadingCorrection(false);
    swerveDrive.setCosineCompensator(false);
    // swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
    swerveDrive.resetOdometry(new Pose2d(1, 1, new Rotation2d()));
  }

  @Override
  public void periodic() {
    swerveDrive.updateOdometry();
    // This method will be called once per scheduler run
    swerveDrive.field.setRobotPose(getPose());
    newField.setRobotPose(getPose());
    SmartDashboard.putData("awdawd", newField);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /** Core drive helper */
  public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    Translation2d translation =
        new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double rotation = speeds.omegaRadiansPerSecond;

    swerveDrive.drive(translation, rotation, fieldRelative, false);
  }

  /** YAGSL's field-oriented helper (expects field-relative speeds) */
  public void driveFieldRelative(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public void resetPose(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public ChassisSpeeds getCurrentSpeeds() {
    return swerveDrive.getRobotVelocity();
  }

  /** Robot-relative teleop drive */
  public Command driveCommand(DoubleSupplier translationX,
                              DoubleSupplier translationY,
                              DoubleSupplier angularRotationX) {
    return run(() -> {
      Translation2d translationCubed = SwerveMath
          .cubeTranslation(new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()));
      translationCubed = translationCubed.times(swerveDrive.getMaximumChassisVelocity());

      double rotation = Math.pow(angularRotationX.getAsDouble(), 3)
          * swerveDrive.getMaximumChassisAngularVelocity();

      SmartDashboard.putNumber("DriveCommand/rot", rotation);
      SmartDashboard.putNumber("DriveCommand/translationX", translationX.getAsDouble());
      SmartDashboard.putNumber("Drivecommand/translationY", translationY.getAsDouble());

      ChassisSpeeds chassisSpeeds =
          new ChassisSpeeds(translationCubed.getX(), translationCubed.getY(), rotation);
      drive(chassisSpeeds, false);
    });
  }

  public void driveTeleopFieldRelative(double x, double y, double rot) {
    // x,y,rot are -1..1 joystick style
    Translation2d translation = SwerveMath
        .cubeTranslation(new Translation2d(x, y))
        .times(swerveDrive.getMaximumChassisVelocity());
  
    double omega = Math.pow(rot, 3) * swerveDrive.getMaximumChassisAngularVelocity();
  
    Rotation2d robotAngle = swerveDrive.getGyro().getRotation3d().toRotation2d();
  
    ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(
        translation.getX(),
        translation.getY(),
        omega,
        robotAngle
    );
  
    drive(robotRelative, false);
  }
  

  /**
   * FIELD-relative teleop drive.
   *
   * translationX: forward/back on FIELD (left stick Y, usually inverted)
   * translationY: left/right on FIELD (left stick X, usually inverted)
   * angularRotationX: rotation command (right stick X)
   */
  public Command driveFieldRelativeCommand(DoubleSupplier translationX,
                                           DoubleSupplier translationY,
                                           DoubleSupplier angularRotationX) {
    return run(() -> {
      // Shape joystick input
      Translation2d translationCubed = SwerveMath
          .cubeTranslation(new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()));
      translationCubed = translationCubed.times(swerveDrive.getMaximumChassisVelocity());

      double rotation = Math.pow(angularRotationX.getAsDouble(), 3)
          * swerveDrive.getMaximumChassisAngularVelocity();

      SmartDashboard.putNumber("DriveCommand/rot", rotation);
      SmartDashboard.putNumber("DriveCommand/translationX", translationX.getAsDouble());
      SmartDashboard.putNumber("DriveCommand/translationY", translationY.getAsDouble());

      // Get current robot heading from the YAGSL gyro
      Rotation2d robotAngle =
          swerveDrive.getGyro().getRotation3d().toRotation2d();

      // Convert FIELD-relative driver commands into ROBOT-relative speeds
      ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          translationCubed.getX(),
          translationCubed.getY(),
          rotation,
          robotAngle);

      // Now drive using ROBOT-relative speeds
      drive(robotRelativeSpeeds, false);
    });
  }
}
