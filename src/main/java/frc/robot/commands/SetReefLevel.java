package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Positions;
import frc.robot.Constants.Elevator.ElevatorPositions;
import frc.robot.Constants.Intake.IntakePositions;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class SetReefLevel extends Command{

    ElevatorSubsystem elevatorSubsystem;
    IntakeSubsystem intakeSubsystem;

    double setpoint;

    private SequentialCommandGroup alternateLevel(double setpoint){
        SequentialCommandGroup sequentialCommandGroup = null;

        switch ((int) setpoint) {

            case (int) Positions.L1_POSITION:

                    sequentialCommandGroup = new SequentialCommandGroup(
                        new IntakePosition(IntakePositions.PUT_CORAL),
                        new ElevatorPosition(ElevatorPositions.HOME),
                        new IntakePosition(IntakePositions.DEFAULT_POSITION)
                    );


            case (int) Positions.L2_POSITION:
            
                    sequentialCommandGroup = new SequentialCommandGroup(
                        new IntakePosition(IntakePositions.PUT_CORAL),
                        new ElevatorPosition(ElevatorPositions.L2)
                    );

                
            case (int) Positions.L3_POSITION:
            
                    sequentialCommandGroup = new SequentialCommandGroup(
                        new IntakePosition(IntakePositions.PUT_CORAL),
                        new ElevatorPosition(ElevatorPositions.L3)
                    );

                
            case (int) Positions.L4_POSITION:
            
                    sequentialCommandGroup = new SequentialCommandGroup(
                        new IntakePosition(IntakePositions.PUT_CORAL),
                        new ElevatorPosition(ElevatorPositions.L4),
                        new IntakePosition(IntakePositions.OPEN_L4)
                    );

        }
        return sequentialCommandGroup;
    }

    public SetReefLevel(double setpoint){
        this.setpoint = setpoint;
        this.intakeSubsystem = IntakeSubsystem.getInstance();
        this.elevatorSubsystem = ElevatorSubsystem.getInstance();
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        alternateLevel(setpoint);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atSetpoint() && intakeSubsystem.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.stopElevator();
        intakeSubsystem.stopIntakeMotor();
    }
}
