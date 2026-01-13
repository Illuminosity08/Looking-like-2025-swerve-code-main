package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  // NetworkTables table for the Limelight
  // If you renamed the Limelight in settings, update "limelight" to that name.
  private final NetworkTable limelightTable;

  public Vision() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  /** Whether Limelight currently sees any target (tv == 1) */
  public boolean hasTarget() {
    return limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
  }

  /** AprilTag ID currently being tracked (tid). Returns -1 if none. */
  public int getTagId() {
    return (int) limelightTable.getEntry("tid").getDouble(-1);
  }

  /** Horizontal offset from crosshair to target, in degrees */
  public double getTx() {
    return limelightTable.getEntry("tx").getDouble(0.0);
  }

  /** Vertical offset from crosshair to target, in degrees */
  public double getTy() {
    return limelightTable.getEntry("ty").getDouble(0.0);
  }

  /** Target area (0–100%) */
  public double getTa() {
    return limelightTable.getEntry("ta").getDouble(0.0);
  }

  /**
   * AprilTag pose relative to robot:
   * [x, y, z, roll, pitch, yaw]
   * Units: meters, degrees
   */
  public double[] getTargetPoseRobotSpace() {
    return limelightTable
        .getEntry("targetpose_robotspace")
        .getDoubleArray(new double[6]);
  }

  @Override
  public void periodic() {
    // Push useful values to SmartDashboard for debugging
    SmartDashboard.putBoolean("LL3A/HasTarget", hasTarget());
    SmartDashboard.putNumber("LL3A/TagID", getTagId());
    SmartDashboard.putNumber("LL3A/tx", getTx());
    SmartDashboard.putNumber("LL3A/ty", getTy());
    SmartDashboard.putNumber("LL3A/ta", getTa());
  }
}