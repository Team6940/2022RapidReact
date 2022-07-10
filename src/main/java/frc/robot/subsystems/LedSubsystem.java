package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.leds.cmdSetLeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.team1323.lib.util.HSVtoRGB;
import frc.robot.lib.team1323.lib.util.MovingAverage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class LedSubsystem  extends SubsystemBase {
    private static LedSubsystem instance = null;
    AddressableLED ledStrip = new AddressableLED(Constants.LED_PORT);
    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(Constants.LED_LENGTH);
    int m_rainbowFirstPixelHue = 0;
    boolean lit = false;
    double lastOnTime = 0.0;
    double lastOffTime = 0.0;
    double transTime = 0.0;
    private State currentState = State.OFF;

    public LedSubsystem() {

        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);
        ledStrip.start();
    }

    public static LedSubsystem getInstance() {
        if (instance == null){
          instance = new LedSubsystem();
        }
        return instance;
      }
    public enum State{
        OFF(0, 0, 0, Double.POSITIVE_INFINITY, 0.0, false),
        DISABLED(255, 20, 30, Double.POSITIVE_INFINITY, 0.0, false), // solid pink
        ENABLED(0, 0, 255, Double.POSITIVE_INFINITY, 0.0, false), // solid blue
        EMERGENCY(255, 0, 0, 0.5, 0.5, false), // blinking red
        HOOD_TUCKED(255, 20, 0, Double.POSITIVE_INFINITY, 0.0, false), // solid pink
        TARGET_VISIBLE(0, 255, 0, Double.POSITIVE_INFINITY, 0.0, false), // solid orange
        TARGET_TRACKING(255, 165, 0, 0.0625, 0.0625, false), // flashing green
        INVISIBLE_TARGET_TRACKING(255, 255, 0, 0.0625, 0.0625, false), // flashing green
        LIMELIGHT_SEES_ONLY(0, 255, 0, Double.POSITIVE_INFINITY, 0.0, false), // solid green
        CLIMB_MODE(255, 0, 255, Double.POSITIVE_INFINITY, 0.0, false), // solid purple
        CLIMB_MODE_BUDDY(255, 0, 255, 0.125, 0.125, false), // flashing purple
        EXTENDING(255, 165, 0, Double.POSITIVE_INFINITY, 0.0, false), // solid orange
        EXTENDING_BUDDY(255, 165, 0, 0.125, 0.125, false), // flashing orange
        HUGGING(255, 255, 0, Double.POSITIVE_INFINITY, 0.0, false), // solid yellow
        HUGGING_BUDDY(255, 255, 0, 0.125, 0.125, false), // flashing yellow
        WRANGLING(0, 255, 255, 0.0625, 0.0625, false), // flashing cyan
        CLIMBING(0, 255, 0, Double.POSITIVE_INFINITY, 0.0, false), // solid green
        CLIMBING_BUDDY(0, 255, 0, 0.125, 0.125, false), // flashing green
        RAINBOW(0, true),
        BREATHING_PINK(357, 10.0, true);

        double red, green, blue, onTime, offTime, cycleTime, transitionTime;
        float startingHue;
        List<List<Double>> colors = new ArrayList<List<Double>>();
        boolean isCycleColors;
        private State(double r, double g, double b, double onTime, double offTime, boolean isCycleColors){
            red = r;
            green = g;
            blue = b;
            this.onTime = onTime;
            this.offTime = offTime;
        }

        private State(float hue, boolean cycle) {
            this.startingHue = hue;
            this.isCycleColors = cycle;
        }

        private State(float hue, double transTime, boolean cycle) {
            this.startingHue = hue;
            this.transitionTime = transTime;
            this.isCycleColors = cycle;
        }

        private State(List<List<Double>> colors, double cycleTime, boolean isCycleColors, double transitionTime) {
            this.colors = colors;
            this.cycleTime = cycleTime;
            this.isCycleColors = isCycleColors;
            this.transitionTime = transitionTime;
        }
    }

    public State getState(){
        return currentState; 
    }
    private void setState(State newState){
        if(newState != currentState){
            currentState = newState;
            lastOffTime = 0.0;
            lastOnTime = 0.0;
            lit = false;
        }
    }

    public void conformToState(State state){
        setState(state);
    }

    public double stateHue = State.RAINBOW.startingHue;
    public float saturation = 1.0f; // Ensures that the colors are on the outside of the color wheel
    public float value = 0.3f; // Hardcoded brightness
    public double startingTransTime = 0.0;
    public boolean resetBreath = false;

    public void writePeriodicOutputs(){
        double timestamp = Timer.getFPGATimestamp();
        if (currentState == State.RAINBOW && currentState.isCycleColors == true) {
            stateHue += 2;
            if (stateHue >= (360 - State.RAINBOW.startingHue)) {
                stateHue = State.RAINBOW.startingHue;
            }

            float rgb[] = new float[3];
            MovingAverage averageR = new MovingAverage(5);
            MovingAverage averageG = new MovingAverage(5);
            MovingAverage averageB = new MovingAverage(5);

            
            if (saturation > 1) {
                saturation = 1;
            }
            if (saturation < 0) {
                saturation = 0;
            }
            if (value > 1) {
                value = 1;
            }
            if (value < 0) {
                value = 0;
            }
            
            rgb = HSVtoRGB.convert(stateHue, saturation, value);

            averageR.add(rgb[0]);
            averageG.add(rgb[1]);
            averageB.add(rgb[2]);
            rgb[0] = (float)averageR.getAverage() * 255;
            rgb[1] = (float)averageG.getAverage() * 255;
            rgb[2] = (float)averageB.getAverage() * 255;

            setLeds((int)rgb[0], (int)rgb[1], (int)rgb[2]);

        } else if (currentState == State.BREATHING_PINK && currentState.isCycleColors == true) {
            if (startingTransTime <= currentState.transitionTime && !resetBreath) {
                startingTransTime += currentState.transitionTime / 50.0;
            } else if (resetBreath) {
                startingTransTime -= currentState.transitionTime / 50.0;
            }
            if (resetBreath && startingTransTime <= 0.0) {
                resetBreath = false;
            } else if (!resetBreath && startingTransTime >= currentState.transitionTime) {
                resetBreath = true;
            }


            float rgb[] = new float[3];
            MovingAverage averageR = new MovingAverage(10);
            MovingAverage averageG = new MovingAverage(10);
            MovingAverage averageB = new MovingAverage(10);

            double valueBasedOnTime = currentState.transitionTime - startingTransTime;
            
            rgb = HSVtoRGB.convert(State.BREATHING_PINK.startingHue, 0.922f, valueBasedOnTime * 0.6);

            averageR.add(rgb[0]);
            averageG.add(rgb[1]);
            averageB.add(rgb[2]);
            rgb[0] = (float)averageR.getAverage() * 255;
            rgb[1] = (float)averageG.getAverage() * 255;
            rgb[2] = (float)averageB.getAverage() * 255;

            setLeds((int)rgb[0], (int)rgb[1], (int)rgb[2]);

        } else if(!lit && (timestamp - lastOffTime) >= currentState.offTime && currentState.isCycleColors == false){
            setLeds((int)currentState.red, (int)currentState.green, (int)currentState.blue);
            lastOnTime = timestamp;
            lit = true;
        } else if(lit && !Double.isInfinite(currentState.onTime) && currentState.isCycleColors == false){
            if((timestamp - lastOnTime) >= currentState.onTime){
                setLeds(0, 0, 0);
                lastOffTime = timestamp;
                lit = false;
            }
        } 
    }


    public void initDefaultCommand() {
        setDefaultCommand(new cmdSetLeds(this));
      }

    public void setLed(int led, int r, int g, int b) {
        ledBuffer.setRGB(led, r, g, b);
        ledStrip.setData(ledBuffer);
    }

    public void setLeds(int r, int g, int b) {
        for(int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
        ledStrip.setData(ledBuffer);
        SmartDashboard.putNumber("Debug/Led/R", r);
        SmartDashboard.putNumber("Debug/Led/G", g);
        SmartDashboard.putNumber("Debug/Led/B", b);
    }
    public void rainbow() {
        // For every pixel
        for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        final var hue = (m_rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength()))% 180;
        // Set the value
        ledBuffer.setHSV(i, hue, 255, 128);
        }
        ledStrip.setData(ledBuffer);
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
    }

}
