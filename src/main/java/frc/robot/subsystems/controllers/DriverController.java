package frc.robot.subsystems.controllers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Controllers;

public class DriverController implements IDDriverController{

    public static DriverController mInstance = null;

    private CommandXboxController controller;

    public static DriverController getInstance(){
        if(mInstance == null){
            return new DriverController();
        }
        return mInstance;
    }
    
    private DriverController(){
        this.controller = new CommandXboxController(0);
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

    public Trigger a(){
        return controller.a();
    }

    public Trigger b(){
        return controller.b();
    }

    public Trigger x(){
        return controller.x();
    }

    public Trigger y(){
        return controller.y();
    }

    public Trigger rightBumper(){
        return controller.rightBumper();
    }

    public Trigger leftBumper(){
        return controller.leftBumper();
    }
}
