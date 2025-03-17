package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Intake extends SubsystemBase {
    private final SparkMax intakeMotor;

    private boolean has_peice = false;

    // Motor Speeds
    private static final double INTAKE_SPEED = -0.6;   // Adjust as needed
    private static final double EJECT_SPEED = 0.5;  // Reverse direction for ejecting

    public Intake(int motorPort) {
        intakeMotor = new SparkMax(motorPort, MotorType.kBrushless);
    }

    public void intake() {
        intakeMotor.set(INTAKE_SPEED);
        has_peice = true;
    }

    public void hold() {
        if (has_peice) {    
            intakeMotor.set(-0.5);
        } else {   
            intakeMotor.set(0);
        }
    }

    public void eject() {
        intakeMotor.set(EJECT_SPEED);
        has_peice = false;
    }

    public void stop() {
        intakeMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Speed", intakeMotor.get());
    }
}
