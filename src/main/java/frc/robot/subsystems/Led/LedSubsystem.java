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

    public static LedSubsystem mInstance = null;

    private LedSubsystem(){
        this.addressableLED = new AddressableLED(0);
        this.buffer = new AddressableLEDBuffer(60);

        this.pattern = LEDPattern.solid(Color.kRed);
        this.pattern.applyTo(buffer);
        
        this.addressableLED.setLength(buffer.getLength());
        this.addressableLED.setData(buffer);
        this.addressableLED.start();
    }

    public static LedSubsystem getInstance(){
        if(mInstance == null){
            mInstance = new LedSubsystem();
        }
        return mInstance;
    }

    public void setColor(Color color){
        this.pattern = LEDPattern.solid(color);
        this.pattern.applyTo(buffer);
        this.addressableLED.setData(buffer);
    }

    public void getRGBColor(int index, int r, int g, int b){
        this.buffer.setRGB(index, r, g, b);
        this.addressableLED.setData(buffer);
    }
}