package frc.robot.commands;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class ResetPigeon extends Command{
    
    Pigeon2 pigeon2;
    SwerveSubsystem subsystem;

    public ResetPigeon(SwerveSubsystem subsystem, Pigeon2 pigeon2){
        this.subsystem = subsystem;
        this.pigeon2 = pigeon2;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        System.out.println("resetando pigeon");
    }

    @Override
    public void execute() {
        try{

            pigeon2.reset();
            subsystem.swerveDrive.zeroGyro();
            
        } catch(Exception e){
            System.out.println("erro ao resetar o pigeon");
        }
    }

    @Override
    public boolean isFinished() {
        return pigeon2.getYaw().getValueAsDouble() == 0.0;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.drive(new Translation2d(0, 0), 0, true);
    }
}
