// package frc.robot.subsystems;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// public class Wrist extends SubsystemBase {
//         private final SparkMax wristMotor;
//     private final SparkMaxConfig wristConfig = new SparkMaxConfig();
//     private final RelativeEncoder wristEncoder;
//     private final PIDController pidController = new PIDController(kP, kI, kD);

//     // PID Constants (Tune as needed)
//     private static final double kP = 0.02;  // Proportional gain
//     private static final double kI = 0.0;   // Integral gain
//     private static final double kD = 0.0;   // Derivative gain

//     private static final double WRIST_UP_POSITION = 0.0;  // Stowed position
//     private static final double WRIST_DOWN_POSITION = 1.0;  // Deployed position

//     public Wrist(int motorPort) {
//         wristMotor = new SparkMax(motorPort, MotorType.kBrushless);
//         wristEncoder = wristMotor.getEncoder();
//         // Zero encoder for consistent positioning
//         wristEncoder.setPosition(0);
//         wristConfig.smartCurrentLimit(40);  // Prevents burning out the motor
//         wristConfig.idleMode(IdleMode.kBrake);  // Holds position
//         wristMotor.configure(wristConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
//     }

//     public void setPower(double power) {
//         wristMotor.set(power);
//     }

//     public void setWristPosition(double position) {
//         pidController.setSetpoint(position);
//     }

//     public void wristUp() {
//         setWristPosition(WRIST_UP_POSITION);
//     }

//     public void wristDown() {
//         setWristPosition(WRIST_DOWN_POSITION);
//     }

//     public void stop() {
//         wristMotor.set(0);  // Stops the motor immediately
//     }
//     public void setElevatorPosition(double targetHeight) {
//         double pidOutput = pidController.calculate(wristEncoder.getPosition(), targetHeight);
//         setWristPosition(pidOutput);
//     }

//     @Override
//     public void periodic() {
//         SmartDashboard.putNumber("Wrist Position", wristEncoder.getPosition());
//     }
// }

