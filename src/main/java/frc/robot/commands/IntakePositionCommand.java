package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePositionCommand extends Command{
    
    IntakeSubsystem intake;
    double setpoint;

    public IntakePositionCommand(IntakeSubsystem intake, double setpoint){
        this.intake = intake;
        this.setpoint = setpoint;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        System.out.println("inicializando");
    }

    @Override
    public void execute() {
        try{
        intake.setPosition(setpoint);
    } catch(Exception e){
        System.out.println("erro ao posicionar o intake: "+ e.getMessage());
    }
}
    @Override
    public boolean isFinished() {
        return intake.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntakeMotor();
    }
}
