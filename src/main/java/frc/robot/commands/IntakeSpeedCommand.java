package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeSpeedCommand extends Command{
    
    IntakeSubsystem intake;
    double speed;
    boolean getCoralBoolean;

    public IntakeSpeedCommand(double speed){
        this(speed, false);
    }

    public IntakeSpeedCommand(boolean getCoralBoolean){
        this(0.2, true);
    }

    private IntakeSpeedCommand(double speed, boolean getCoralBoolean){
        this.intake = IntakeSubsystem.getInstance();
        this.speed = speed;
        this.getCoralBoolean = getCoralBoolean;
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
        return false || (getCoralBoolean == true && intake.IsTouched() == true);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopCoralMotor();
    }
}
