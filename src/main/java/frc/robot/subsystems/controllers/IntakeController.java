package frc.robot.subsystems.controllers;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Controllers;

public class IntakeController implements IDIntakeController{
    
    public CommandXboxController controller;

    public static IntakeController mInstance = null;

    public static IntakeController getInstance(){
        if(mInstance == null){
            return new IntakeController();
        }
        return mInstance;
    }

    private IntakeController(){
        this.controller = new CommandXboxController(Controllers.INTAKE_CONTROLLER);
    }

    @Override
    public Trigger getCoralButton(){
        return controller.rightBumper();
    }

    @Override
    public Trigger throwCoralOnIntake(){
        return controller.leftBumper();
    }

    @Override
    public Trigger L1Button(){
        return controller.a();
    }

    @Override
    public Trigger L2Button(){
        return controller.b();
    }

    @Override
    public Trigger L3Button(){
        return controller.y();
    }

    @Override
    public Trigger L4Button(){
        return controller.x();
    }

    @Override
    public Trigger ProcessorButton(){
        return controller.start();
    }

    @Override
    public Trigger L2Algae(){
        return controller.rightBumper();
    }

    @Override
    public Trigger L3Algae(){
        return controller.leftBumper();
    }

    @Override
    public double UpElevator() {
       return controller.getRightY();
    }

}
