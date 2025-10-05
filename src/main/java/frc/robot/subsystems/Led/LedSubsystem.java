package frc.robot.subsystems.Led;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase{

    private AddressableLED addressableLED;
    private AddressableLEDBuffer buffer;

    private LEDPattern pattern;

    public static LedSubsystem mInstance = null;

    private LedSubsystem(){
        this.addressableLED = new AddressableLED(9);
        this.buffer = new AddressableLEDBuffer(60);
        this.pattern = LEDPattern.solid(Color.kBlack);

        this.addressableLED.setLength(this.buffer.getLength());
        this.addressableLED.start();
    }
    
    public static LedSubsystem getInstance(){
        if(mInstance == null){
            mInstance = new LedSubsystem();
        }
        return mInstance;
    }

    @Override
    public void periodic() {
        if(this.pattern != null){
            pattern.applyTo(buffer);
            addressableLED.setData(buffer);
        }
    }

    public LEDPattern getActualPattern(){
        return pattern;
    }

    public Command runPattern() {
        return run(() -> {
            if (this.pattern != null) {
                this.pattern.applyTo(buffer);
                this.addressableLED.setData(buffer);
            }
        });
    }

    public void setPattern(LEDPattern pattern){
        this.pattern = pattern;
    }

    public void setSolidColor(Color color){
        setPattern(LEDPattern.solid(color));
    }

    public void setRGBColor(int index, int r, int g, int b){
        for(int i = 0; i <= index; i++){
            this.buffer.setRGB(index, r, g, b);
            this.addressableLED.setData(buffer);
      }
    }

    public void setRainbow() {
        Distance ledSpace = Meters.of(1.0 / 60.0);
        LEDPattern rainbowPattern = LEDPattern.rainbow(255, 128);
        LEDPattern scrollingRainbowPattern = 
            rainbowPattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), ledSpace);

        scrollingRainbowPattern.applyTo(buffer);
    }
}