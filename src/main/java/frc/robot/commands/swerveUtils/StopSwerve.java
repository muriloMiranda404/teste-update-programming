package frc.robot.commands.swerveUtils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class StopSwerve extends Command{
    
    SwerveSubsystem subsystem;

    public StopSwerve(){
        this.subsystem = SwerveSubsystem.getInstance();

        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        try{
            subsystem.drive(new Translation2d(0, 0), 0, true);
        } catch(Exception e){
            System.out.println("erro ao para swerve");
        }
    }

    @Override
    public boolean isFinished() {
        return subsystem.swerveIsStoped();
    }

    @Override
    public void end(boolean interrupted) {}
}
