package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSpeedCommand extends Command{
    
    IntakeSubsystem intake;
    double speed;
    boolean stop;

    public IntakeSpeedCommand(IntakeSubsystem intake, double speed){
        this(intake, speed, false);
    }

    public IntakeSpeedCommand(IntakeSubsystem intake, boolean stop){
        this(intake, 0.2, true);
    }

    private IntakeSpeedCommand(IntakeSubsystem intake, double speed, boolean stop){
        this.intake = intake;
        this.speed = speed;
        this.stop =  stop;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        System.out.println("velocidade do motor: " + speed);
    }

    @Override
    public void execute() {
        try{
        intake.setSpeed(speed);

    } catch(Exception e){
        System.out.println("erro ao colocar velocidade");
    }
}
    @Override
    public boolean isFinished() {
        return false || (stop == true && intake.IsTouched() == true);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopCoralMotor();
    }
}
