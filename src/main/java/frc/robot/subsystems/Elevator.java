package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;



public class Elevator extends SubsystemBase {
    private final SparkMax elevatorMotor;
    private final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    private final RelativeEncoder elevatorEncoder;
    private final PIDController pidController;

    private static final double kP = 0.5;  // Tune these values for PID control
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double MAX_HEIGHT = 1.5;  // Max height in meters (example)
    private static final double MIN_HEIGHT = 0.0;  // Min height

    public Elevator() {
        elevatorMotor = new SparkMax(Constants.ELEVATOR_ID, MotorType.kBrushless);
        pidController = new PIDController(kP, kI, kD);
        elevatorEncoder = elevatorMotor.getEncoder();
        elevatorEncoder.setPosition(0);
        elevatorConfig.smartCurrentLimit(40);  // Prevents burning out the motor
        elevatorConfig.idleMode(IdleMode.kBrake);  // Holds position
        elevatorMotor.configure(elevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    public void moveToPosition(double position) {
        setElevatorPosition(position);
    }
    
    public void setElevatorSpeed(double speed) {
        double currentHeight = elevatorEncoder.getPosition();  

        // Safety limits
        if ((currentHeight >= MAX_HEIGHT && speed > 0) || (currentHeight <= MIN_HEIGHT && speed < 0)) {
            speed = 0;  // Stop motor if at limits
        }

        elevatorMotor.set(speed);
        SmartDashboard.putNumber("Elevator Height", currentHeight);
    }

    public void setElevatorPosition(double targetHeight) {
        double pidOutput = pidController.calculate(elevatorEncoder.getPosition(), targetHeight);
        setElevatorSpeed(pidOutput);
    }

    public double getHeight() {
        return elevatorEncoder.getPosition();
    }
}
