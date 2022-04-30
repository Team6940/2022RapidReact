package frc.robot.lib.team6940.math;

public class MathUtils {

    private MathUtils() {}

    /**
     * Calculate the arccot angle
     * 
     * @param angle The first angle
     * @return The arccot of the angle 
     */
    public static double acot(double angle){
        return Math.acos(angle) / Math.asin(angle);
    }
    
}
