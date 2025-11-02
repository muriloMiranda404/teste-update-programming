package frc.FRC9485.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.FRC9485.utils.Util;
import frc.FRC9485.constants.JoystickConstants;

public class MechanismJoystick implements IDMechanism{
    
    private CommandXboxController controller;
    public double setpoint;

    public static MechanismJoystick mInstance = null;

    private MechanismJoystick(){
        this.controller = new CommandXboxController(JoystickConstants.INTAKE_CONTROLLER);
    }
    
    public static MechanismJoystick getInstance(){
        if(mInstance == null){
            mInstance =  new MechanismJoystick();
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
        return L2Button().and(L3Algae());
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

    @Override
    public double getSetpoint(){
        return this.setpoint;
    }

    @Override
    public boolean joystickIsNothingUsingMechanism() {
        return Util.inRange(controller.getRightY(), -JoystickConstants.DEADBAND, JoystickConstants.DEADBAND) 
        && Util.inRange(controller.getRightX(), -JoystickConstants.DEADBAND, JoystickConstants.DEADBAND);
    }

    @Override
    public double getRightTrigger(){
        return controller.getRightTriggerAxis();
    }

    @Override
    public double getLeftTrigger(){
        return controller.getLeftTriggerAxis();
    }

    public double getRightY(){
        return controller.getRightY();
    }
}
