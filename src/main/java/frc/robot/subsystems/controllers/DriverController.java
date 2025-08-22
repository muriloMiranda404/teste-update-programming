package frc.robot.subsystems.controllers;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Controllers;
import frc.robot.subsystems.swerve.SwerveSubsystem;

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
        this.controller = new CommandXboxController(Controllers.DRIVE_CONTROLLER);
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
    public double automaticInverted(double value){
        Alliance invert = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Red;

        if(invert == Alliance.Red){
            return -value;
        }

        return value;
    }
    
    @Override
    public Trigger TurboMode(){
        return controller.rightTrigger(0.8);
    }

    @Override
    public double getMarcha(){
        return controller.getRightTriggerAxis() - controller.getLeftTriggerAxis();
    }

    @Override
    public Command driverRobot(){

        double marcha = 1.0;
        if(TurboMode().getAsBoolean()){
            marcha = controller.getRightTriggerAxis() - controller.getLeftTriggerAxis() + 0.8;
        } else if(marcha < 0){
            marcha *= -1.0;
        } else {
            marcha = 1;
        }
        
        // double invert = automaticInverted() == true ? -1.0 : 1.0;
        
        // if(automaticInverted()){
        //     invert = -1;
        // } else{
        //     invert = 1;
        // }
        
        // double leftY = controller.getLeftY() * marcha * invert;
        // double leftX= controller.getLeftX() * marcha * invert;
        // double rightX = controller.getRightX() * marcha; 

        // Command drive = swerveSubsystem.driveCommand(
        //     () -> MathUtil.applyDeadband(leftY, Controllers.DEADBAND),
        //     () -> MathUtil.applyDeadband(leftX, Controllers.DEADBAND),
        //     () -> MathUtil.applyDeadband(rightX, Controllers.DEADBAND));

            return null;
    }

    @Override
    public Trigger alingRobotOnReef() {
        return controller.pov(270);
    }

    @Override
    public double getLeftX(){
        if(TurboMode().getAsBoolean()){
           return MathUtil.applyDeadband(automaticInverted(controller.getLeftX()), Controllers.DEADBAND);
        } 
        else if(slowMode().getAsBoolean()){
            return MathUtil.applyDeadband(automaticInverted(controller.getLeftX()), Controllers.DEADBAND) * 0.2;
        }
        return MathUtil.applyDeadband(automaticInverted(controller.getLeftX()), Controllers.DEADBAND) * 0.6;
    }

    @Override
    public double getLeftY(){
        if(TurboMode().getAsBoolean()){
            return MathUtil.applyDeadband(automaticInverted(controller.getLeftY()), Controllers.DEADBAND);
        }
        else if(slowMode().getAsBoolean()){
            return MathUtil.applyDeadband(automaticInverted(controller.getLeftY()), Controllers.DEADBAND) * 0.2;
        }
        return MathUtil.applyDeadband(automaticInverted(controller.getLeftX()), Controllers.DEADBAND) * 0.6;
    }

    @Override
    public double getRightX(){
        if(TurboMode().getAsBoolean()){
            return MathUtil.applyDeadband(automaticInverted(controller.getRightX()), Controllers.DEADBAND);
        }
        else if(slowMode().getAsBoolean()){
            return MathUtil.applyDeadband(automaticInverted(controller.getRightX()), Controllers.DEADBAND) * 0.2;
        }
        return MathUtil.applyDeadband(automaticInverted(controller.getRightX()), Controllers.DEADBAND) * 0.6;
    }

    @Override
    public double getRightY(){
        if(TurboMode().getAsBoolean()){
            return MathUtil.applyDeadband(automaticInverted(controller.getRightY()), Controllers.DEADBAND);
        }
        else if(slowMode().getAsBoolean()){
            return MathUtil.applyDeadband(automaticInverted(controller.getRightY()), Controllers.DEADBAND) * 0.2;
        }
        return MathUtil.applyDeadband(automaticInverted(controller.getRightY()), Controllers.DEADBAND) * 0.6;
    }

    @Override
    public Trigger slowMode() {
        return controller.leftTrigger(0.8);
    }
}
