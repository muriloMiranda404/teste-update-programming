package tests;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Controllers;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class TeleopSwerve extends Command{
    
    CommandXboxController controller;
    SwerveSubsystem swerveSubsystem;

    DoubleSupplier X;
    DoubleSupplier Y;
    DoubleSupplier rotation;

    double flip = 1;

    double normal_speed = 2;
    double normal_rotation = 2;

    public TeleopSwerve(CommandXboxController controller){
        this.controller = controller;
        this.swerveSubsystem = SwerveSubsystem.getInstance();
        this.X = () -> MathUtil.applyDeadband(controller.getLeftY() * flip, Controllers.DEADBAND);
        this.Y = () -> MathUtil.applyDeadband(controller.getLeftX() * flip, Controllers.DEADBAND);
        this.rotation = () -> MathUtil.applyDeadband(controller.getRightX() * flip, Controllers.DEADBAND);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        flip = DriverStation.getAlliance().get() == Alliance.Red ? -1 : 1;
    }

    @Override
    public void execute() {
        swerveSubsystem.drive(new Translation2d(X.getAsDouble(), Y.getAsDouble()), rotation.getAsDouble(), true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
