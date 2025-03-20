package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.parser.PIDFConfig;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Lift extends SubsystemBase {
        private final SparkMax liftMotor;
    private final SparkMaxConfig liftConfig = new SparkMaxConfig();
    private final RelativeEncoder liftEncoder;
    private final PIDController pidController = new PIDController(kP, kI, kD);

    // PID Constants (Tune as needed)
    private static final double kP = 0.5;  // Proportional gain
    private static final double kI = 0.0;   // Integral gain
    private static final double kD = 0.0;   // Derivative gain

    private static final double LIFT_UP_POSITION = 0.0;  // Stowed position
    private static final double LIFT_DOWN_POSITION = 1.0;  // Deployed position

    public Lift(int motorPort) {
        liftMotor = new SparkMax(motorPort, MotorType.kBrushless);
        liftEncoder = liftMotor.getEncoder();
        // Zero encoder for consistent positioning
        liftEncoder.setPosition(1.3);
        liftConfig.smartCurrentLimit(40);  // Prevents burning out the motor
        liftConfig.idleMode(IdleMode.kBrake);  // Holds position
        liftMotor.configure(liftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        pidController.setSetpoint(0);
    }

    public void setPower(double power) {
        liftMotor.set(power);
    }

    public void setLiftPosition(double position) {
        pidController.setSetpoint(position);
    }

    public void liftUp() {
        setLiftPosition(LIFT_UP_POSITION);
    }

    public void liftDown() {
        setLiftPosition(LIFT_DOWN_POSITION);
    }

    public void stop() {
        liftMotor.set(0);  // Stops the motor immediately
    }
    public void calculatePidPower() {
        double pidOutput = pidController.calculate(liftEncoder.getPosition());
        // setLiftPosition(pidOutput);
        liftMotor.set(pidOutput);
        SmartDashboard.putNumber("Lift Pid Power", pidOutput);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Lift Position", liftEncoder.getPosition());
    }
}

