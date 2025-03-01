package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.Elevator;

public class MoveElevator extends Command {
    private final Elevator elevator;
    private final DoubleSupplier speedSupplier;

    public MoveElevator(Elevator elevator, DoubleSupplier speedSupplier) {
        this.elevator = elevator;
        this.speedSupplier = speedSupplier;
        addRequirements(elevator);
    }

    @Override
    public void execute() {
        double speed = speedSupplier.getAsDouble();
        elevator.setElevatorSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setElevatorSpeed(0);  // Stop motor when command ends
    }
}
