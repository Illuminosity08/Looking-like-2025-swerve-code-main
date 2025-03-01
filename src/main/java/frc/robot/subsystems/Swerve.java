// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.telemetry.SwerveDriveTelemetry;

public class Swerve extends SubsystemBase {
  double maximumSpeed = Units.feetToMeters(4.5);
  File directory = new File(Filesystem.getDeployDirectory(), "swerve");
  SwerveDrive swerveDrive;

  /** Creates a new ExampleSubsystem. */
  public Swerve() {

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
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(translation, rotation, fieldRelative, false);
  }

  public void driveFieldRelative(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public void resetPose(Pose2d pose) {
    swerveDrive.resetOdometry(pose);
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier angularRotationX) {
    return run(() -> {
      Translation2d translationCubed = SwerveMath
          .cubeTranslation(new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()));
      translationCubed = translationCubed.times(swerveDrive.getMaximumChassisVelocity());

      double rotation = Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity();

      SmartDashboard.putNumber("DriveCommand/rot", rotation);
      SmartDashboard.putNumber("DriveCommand/translationX", translationX.getAsDouble());
      SmartDashboard.putNumber("Drivecommand/translationY", translationY.getAsDouble());


      drive(translationCubed, rotation, true);
    });
  }
}
