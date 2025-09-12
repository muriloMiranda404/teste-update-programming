package frc.robot.subsystems.Led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase{

    private AddressableLED addressableLED;
    private AddressableLEDBuffer buffer;

    private LEDPattern pattern;

    public LedSubsystem(){
        this.addressableLED = new AddressableLED(0);
        this.buffer = new AddressableLEDBuffer(60);

        this.pattern = LEDPattern.solid(Color.kRed);
        this.pattern.applyTo(buffer);
        
        this.addressableLED.setLength(buffer.getLength());
        this.addressableLED.setData(buffer);
        this.addressableLED.start();
    }

    public void setColor(Color color){
        this.pattern = LEDPattern.solid(color);
        this.pattern.applyTo(buffer);
        this.addressableLED.setData(buffer);
    }
}