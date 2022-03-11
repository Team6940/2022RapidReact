// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class RevColorSensor extends SubsystemBase {
  /** Creates a new RevColorSensor. */
  public final ColorSensorV3 m_ColorSensor;
  public final ColorMatch m_ColorMatcher;

  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);

  // set color inits
  public Color detectedColor;
  
  public RevColorSensor() {
    m_ColorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    m_ColorMatcher = new ColorMatch();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Color detectedColor = m_ColorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    ColorMatchResult match = m_ColorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

  }
}
