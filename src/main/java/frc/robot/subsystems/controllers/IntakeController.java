package frc.robot.subsystems.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Controllers;
import frc.robot.Constants.Positions;

public class IntakeController implements IDIntakeController{
    
    private CommandXboxController controller;
    public double setpoint;

    public static IntakeController mInstance = null;

    private IntakeController(){
        this.controller = new CommandXboxController(Controllers.INTAKE_CONTROLLER);
    }
    
    public static IntakeController getInstance(){
        if(mInstance == null){
            mInstance =  new IntakeController();
        }
        return mInstance;
    }

    @Override
    public Trigger getCoralButton(){
        return controller.povDown();
    }

    @Override
    public XboxController getHID(){
        return controller.getHID();
    }

    @Override
    public Trigger throwCoralOnIntake(){
        return controller.povCenter();
    }

    @Override
    public Trigger L1Button(){
        this.setpoint = Positions.L1_POSITION;
        return controller.a();
    }

    @Override
    public Trigger L2Button(){
        this.setpoint = Positions.L2_POSITION;
        return controller.b();
    }

    @Override
    public Trigger L3Button(){
        this.setpoint = Positions.L3_POSITION;
        return controller.y();
    }

    @Override
    public Trigger L4Button(){
        this.setpoint = Positions.L4_POSITION;
        return controller.x();
    }

    @Override
    public Trigger ProcessorButton(){
        this.setpoint = Positions.PROCESSADOR;
        return controller.start();
    }

    @Override
    public Trigger L2Algae(){
        this.setpoint = Positions.ALGAE_L2;
        return controller.rightBumper();
    }

    @Override
    public Trigger L3Algae(){
        this.setpoint = Positions.ALGAE_L3;
        return controller.leftBumper();
    }

    @Override
    public double UpElevator() {
       return controller.getRightY();
    }

    @Override
    public double getSetpoint(){
        return this.setpoint;
    }
}
