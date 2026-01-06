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
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.imu.ADXRS450Swerve;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.telemetry.SwerveDriveTelemetry;

public class Swerve extends SubsystemBase {
  double maximumSpeed = Units.feetToMeters(2);
  File directory = new File(Filesystem.getDeployDirectory(), "swerve");
  SwerveDrive swerveDrive;
  Field2d newField = new Field2d();

  /** Creates a new ExampleSubsystem. */
  public Swerve() {

    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> drive(speeds, false), // Method that will drive the robot given ROBOT RELATIVE
                                                        // ChassisSpeeds. Also optionally outputs individual module
                                                        // feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic
                                        // drive trains
            new PIDConstants(0.10, 0.0, 0.0
            ), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
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

  public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    Translation2d translation = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    double rotation = speeds.omegaRadiansPerSecond;

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

  public ChassisSpeeds getCurrentSpeeds() {
    return swerveDrive.getRobotVelocity();
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

      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(translationCubed.getX(), translationCubed.getY(), rotation);
      drive(chassisSpeeds, false);
      // driveFieldRelative(ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, swerveDrive.getGyro().getRotation3d().toRotation2d()));
    });
  }

  public Command driveFieldRelativeCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX, DoubleSupplier rotationOffset) {
     return run(() -> {
        Translation2d translationCubed = SwerveMath
            .cubeTranslation(new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()));
        translationCubed = translationCubed.times(swerveDrive.getMaximumChassisVelocity());

        double rotation = Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity();

        SmartDashboard.putNumber("DriveCommand/rot", rotation);
        SmartDashboard.putNumber("DriveCommand/translationX", translationX.getAsDouble());
        SmartDashboard.putNumber("DriveCommand/translationY", translationY.getAsDouble());
        SmartDashboard.putNumber("DriveCommand/rotationOffset", rotationOffset.getAsDouble());

        // double x = translationCubed.getX();
        // double y = translationCubed.getY();
        // double r = rotationOffset.getAsDouble();

        // double dxx = x * Math.cos(r);
        // double dxy = y * Math.cos(r);

        // double dyy = y * Math.cos(r);
        // double dyx = x * Math.cos(r);

        // double dx = MathMath.pow(dxx, 2);

        // // double dx = Math.copySign(Math.sqrt(Math.copySign(Math.pow(translationCubed.getX() * Math.cos(rotationOffset.getAsDouble()), 2), translationCubed.getX() * Math.cos(rotationOffset.getAsDouble())) + Math.copySign(Math.pow(translationCubed.getX() * Math.cos(rotationOffset.getAsDouble()), 2), translationCubed.getX() * Math.cos(rotationOffset.getAsDouble()))), Math.copySign(Math.pow(translationCubed.getX() * Math.cos(rotationOffset.getAsDouble()), 2), translationCubed.getX() * Math.cos(rotationOffset.getAsDouble())) + Math.copySign(Math.pow(translationCubed.getX() * Math.cos(rotationOffset.getAsDouble()), 2), translationCubed.getX() * Math.cos(rotationOffset.getAsDouble())));
        // double dy = Math.sqrt(Math.pow(translationCubed.getY() * Math.cos(rotationOffset.getAsDouble()), 2) + Math.pow(translationCubed.getX() * Math.sin(rotationOffset.getAsDouble()), 2));

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(translationCubed.getX(), translationCubed.getY(), rotation), new Rotation2d(rotationOffset.getAsDouble()));
        drive(chassisSpeeds, false);
      // driveFieldRelative(ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeeds, swerveDrive.getGyro().getRotation3d().toRotation2d()));
    
       });
}

}
