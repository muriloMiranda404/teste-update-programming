package frc.robot.subsystems.controllers;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Controllers;
import frc.robot.subsystems.SwerveSubsystem;

public class DriverController implements IDDriverController{

    public static DriverController mInstance = null;

    private CommandXboxController controller;
    private SwerveSubsystem swerveSubsystem;

    public static DriverController getInstance(){
        if(mInstance == null){
            return new DriverController();
        }
        return mInstance;
    }
    
    private DriverController(){
        this.controller = new CommandXboxController(0);
        this.swerveSubsystem = SwerveSubsystem.getInstance();
    }

    public double ConfigureInputs(boolean active, int choose){

        double marcha;
        double invert = DriverStation.getAlliance().get() == Alliance.Red ? -1.0 : 1.0;
        double gatilho = 0.7 + (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis());

        if(active == true){ 

            marcha = gatilho;

            if(marcha <= 0){
                marcha *= -1.0;
            }

            switch (choose) {
                case 1:
                    
                    return controller.getLeftY() * invert * marcha;
            
                case 2:
                
                    return controller.getLeftX() * invert  * marcha;

                case 3:
                
                    return controller.getRightX() * marcha;
            }
        }
        return choose;
    }


    @Override
    public Trigger a(){
        return controller.a();
    }

    @Override
    public Trigger b(){
        return controller.b();
    }

    @Override
    public Trigger x(){
        return controller.x();
    }

    @Override
    public Trigger y(){
        return controller.y();
    }

    @Override
    public Trigger rightBumper(){
        return controller.rightBumper();
    }

    @Override
    public Trigger leftBumper(){
        return controller.leftBumper();
    }

    @Override
    public boolean automaticInverted(){
        return DriverStation.getAlliance().get() == Alliance.Red;
    }
    
    @Override
    public boolean activateMarcha(){
        return controller.getRightTriggerAxis() - controller.getLeftTriggerAxis() != 0;
    }

    @Override
    public Command driverRobot(){

        double marcha = 1.0;
        if(activateMarcha()){
            marcha = controller.getRightTriggerAxis() - controller.getLeftTriggerAxis() + 0.8;
        } else if(marcha < 0){
            marcha *= -1.0;
        } else {
            marcha = 1;
        }
        
        double invert = 1;
        
        if(automaticInverted()){
            invert = -1;
        } else{
            invert = 1;
        }
        
        double leftY = controller.getLeftY() * marcha * invert;
        double leftX= controller.getLeftX() * marcha * invert;
        double rightX = controller.getRightX() * marcha; 

        Command drive = swerveSubsystem.driveCommand(
            () -> MathUtil.applyDeadband(leftY, Controllers.DEADBAND),
            () -> MathUtil.applyDeadband(leftX, Controllers.DEADBAND),
            () -> MathUtil.applyDeadband(rightX, Controllers.DEADBAND));

            return drive;
    }

    @Override
    public Trigger alingRobotOnReef() {
        return controller.pov(270);
    }

}
