package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ColorConstants;
import edu.wpi.first.wpilibj.RobotBase;

public class ColorSensor extends SubsystemBase {
    private static ColorSensor instance = null;
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kMXP);
    private final ColorMatch m_colorMatcher = new ColorMatch();

    private ColorMatchResult m_match;
    private int testMode1 = 0;

    public ColorSensor() {
        m_colorMatcher.addColorMatch(ColorConstants.kBlueTarget);
        m_colorMatcher.addColorMatch(ColorConstants.kRedTarget);
        m_colorMatcher.addColorMatch(ColorConstants.kNoTarget);

    }

    public static ColorSensor getInstance() {
        if (instance == null){
            instance = new ColorSensor();
        }
        return instance;
    }

    public double[] getBallColor() {
        double[] ret = { m_colorSensor.getRed(), m_colorSensor.getGreen(), m_colorSensor.getBlue() };
        return ret;
    }

    @Override
    public void periodic() {

        Color detectedColor = m_colorSensor.getColor();

        m_match = m_colorMatcher.matchClosestColor(detectedColor);

        /**
         * Open Smart Dashboard or Shuffleboard to see the color detected by the
         * sensor.
         */
        SmartDashboard.putNumber("Debug/colorSensor/Red", detectedColor.red);
        SmartDashboard.putNumber("Debug/colorSensor/Green", detectedColor.green);
        SmartDashboard.putNumber("Debug/colorSensor/Blue", detectedColor.blue);
        SmartDashboard.putNumber("Debug/colorSensor/Confidence", m_match.confidence);
        SmartDashboard.putNumber("Debug/colorSensor/Prox", m_colorSensor.getProximity());

        if (m_match.color == ColorConstants.kRedTarget) {
            SmartDashboard.putString("Debug/colorSensor/Detected Color", "Red");
        } else if (m_match.color == ColorConstants.kBlueTarget) {
            SmartDashboard.putString("Debug/colorSensor/Detected Color", "Blue");
        } else if (m_match.color == ColorConstants.kNoTarget) {
            SmartDashboard.putString("Debug/colorSensor/Detected Color", "Neither");
        }

    }

    public boolean isWrongBall() {
        if (RobotBase.isSimulation()){
            return (testMode1 == 0) ? false:true;
        }

        if(m_match == null) return false;
        boolean prox = m_colorSensor.getProximity() >= 80;
        boolean red = m_match.color == ColorConstants.kRedTarget;
        boolean blue = m_match.color ==ColorConstants.kBlueTarget;
        if (DriverStation.getAlliance().equals(Alliance.Red) && blue && prox) {
            return true;
        } else if (DriverStation.getAlliance().equals(Alliance.Blue) && red && prox) {
            return true;
        } else {
            return false;
        }
    }

    public void DotestMode(){
        testMode1 = (testMode1 == 0) ? 1:0;
    }
}