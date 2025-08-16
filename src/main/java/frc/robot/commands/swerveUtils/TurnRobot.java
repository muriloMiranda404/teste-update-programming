package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class TurnRobot extends Command{
    
    SwerveSubsystem swerve;
    Pigeon2 pigeon2;
    double angulo;

    PIDController controller;

    public TurnRobot(SwerveSubsystem swerve, Pigeon2 pigeon2, double angulo){
        this.swerve = swerve;
        this.pigeon2 = pigeon2;
        this.angulo = angulo;
        this.controller = new PIDController(0.01, 0, 0);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        System.out.println("mudando posição para: " +  angulo);
        controller.setTolerance(1.0);
    }

    @Override
    public void execute() {
        
        try{
        double atual = pigeon2.getYaw().getValueAsDouble();
        double output = controller.calculate(atual, angulo);

        if(atual == angulo || atual > angulo){
            swerve.drive(new Translation2d(0, 0), 0, true);
        }

        swerve.drive(new Translation2d(0, 0), output, true);
    } catch(Exception e){
        System.out.println("erro o mudar angulação: " + e.getMessage());
    }
}

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0, true);
    }
}
