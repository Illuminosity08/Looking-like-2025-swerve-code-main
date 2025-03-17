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

public class Arm extends SubsystemBase {
        private final SparkMax armMotor;
    private final SparkMaxConfig armConfig = new SparkMaxConfig();
    private final RelativeEncoder armEncoder;
    private final PIDController pidController = new PIDController(kP, kI, kD);

    // PID Constants (Tune as needed)
    private static final double kP = 0.5;  // Proportional gain
    private static final double kI = 0.0;   // Integral gain
    private static final double kD = 0.0;   // Derivative gain

    private static final double ARM_UP_POSITION = 0.0;  // Stowed position
    private static final double ARM_DOWN_POSITION = 1.0;  // Deployed position

    public Arm(int motorPort) {
        armMotor = new SparkMax(motorPort, MotorType.kBrushless);
        armEncoder = armMotor.getEncoder();
        // Zero encoder for consistent positioning
        armEncoder.setPosition(1.3);
        armConfig.smartCurrentLimit(40);  // Prevents burning out the motor
        armConfig.idleMode(IdleMode.kBrake);  // Holds position
        armMotor.configure(armConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        pidController.setSetpoint(0);
    }

    public void setPower(double power) {
        armMotor.set(power);
    }

    public void setArmPosition(double position) {
        pidController.setSetpoint(position);
    }

    public void armUp() {
        setArmPosition(ARM_UP_POSITION);
    }

    public void armDown() {
        setArmPosition(ARM_DOWN_POSITION);
    }

    public void stop() {
        armMotor.set(0);  // Stops the motor immediately
    }
    public void calculatePidPower() {
        double pidOutput = pidController.calculate(armEncoder.getPosition());
        // setArmPosition(pidOutput);
        armMotor.set(pidOutput);
        SmartDashboard.putNumber("Arm Pid Power", pidOutput);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Position", armEncoder.getPosition());
    }
}

