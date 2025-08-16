package frc.robot.commands.level.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorPosition extends Command{

    ElevatorSubsystem elevator;
    double setpoint;

    public ElevatorPosition(double setpoint){
        this.elevator = ElevatorSubsystem.getInstance();
        this.setpoint = setpoint;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        elevator.setElevatorPosition(setpoint);
    }

    @Override
    public boolean isFinished() {
        return elevator.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopElevator();
    }
}
