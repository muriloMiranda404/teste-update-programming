package frc.FRC9485.controllers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.FRC9485.utils.Util;
import frc.robot.Constants.Controllers;

public class DriverController implements IDDriverController{

    public static DriverController mInstance = null;

    private CommandXboxController controller;
    private double invert;

    private DriverController(){
        this.controller = new CommandXboxController(Controllers.DRIVE_CONTROLLER);
        invert = 1;
    }
    
    public static DriverController getInstance(){
        if(mInstance == null){
            mInstance = new DriverController();
        }
        return mInstance;
    }

    @Override
    public double Invert(){
        return invert *= -1.0;
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
            marcha = Math.abs(controller.getRightTriggerAxis() - controller.getLeftTriggerAxis());
        }
        return Commands.none();
    }

    @Override
    public Trigger alingRobotOnReef() {
        return controller.pov(0);
    }

    @Override
    public double getLeftX(){
        if(TurboMode().getAsBoolean()){
            return controller.getLeftX() * 1;
        } else if(slowMode().getAsBoolean()){
            return controller.getLeftX() * 0.2;
        } else{
            return controller.getLeftX() * 0.7;
        }
    }

    @Override
    public double getLeftY(){
        if(TurboMode().getAsBoolean()){
            return controller.getLeftY() * 1.0;
        } else if(slowMode().getAsBoolean()){
            return controller.getLeftY() * 0.2;
        } else{
            return controller.getLeftY() * 0.7;
        }
    }

    @Override
    public double getRightX(){
        if(TurboMode().getAsBoolean()){
            return controller.getRightX() * 1.0;
        } else if(slowMode().getAsBoolean()){
            return controller.getRightX() * 0.2;
        } else{
            return controller.getRightX() * 0.7;
        }
    }

    @Override
    public double getRightY(){
        if(TurboMode().getAsBoolean()){
            return controller.getRightY() * 1.0;
        } else if(slowMode().getAsBoolean()){
            return controller.getRightY() * 0.2;
        } else{
            return controller.getRightY() * 0.7;
        }
    }

    @Override
    public Trigger slowMode() {
        return controller.leftTrigger(0.8);
    }

    @Override
    public Trigger emergencyInvert(){
        return controller.start();
    }

    @Override
    public double getPerformByAlliance(double value) {
        var alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Red;

        if(alliance == Alliance.Red){
            return value *= -1;
        } else{
            return value;
        }
    }

    @Override
    public boolean joystickIsNothingUsingDrive() {
        return Util.inRange(getLeftY(), -Controllers.DEADBAND, Controllers.DEADBAND)
        && Util.inRange(getLeftX(), -Controllers.DEADBAND, Controllers.DEADBAND);
    }

    @Override
    public Trigger resetPigeon() {
        return controller.button(10);
    }
}
