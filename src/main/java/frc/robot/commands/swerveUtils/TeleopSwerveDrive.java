package frc.robot.commands.swerveUtils;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FRC9485.controllers.DriverController;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class TeleopSwerveDrive extends Command{
    
    private DriverController driverController;
    private SwerveSubsystem swerveSubsystem;

    private BooleanSupplier fieldOriented;

    public TeleopSwerveDrive(DriverController driverController, BooleanSupplier fieldOriented){
        this.driverController = driverController;
        this.fieldOriented = fieldOriented;
        this.swerveSubsystem = SwerveSubsystem.getInstance();

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() { }

    @Override
    public void execute() {
        swerveSubsystem.driveRobot(
            () -> driverController.getLeftY(), 
            () -> driverController.getLeftX(), 
            () -> driverController.getRightX(), 
            fieldOriented);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) { }
}
