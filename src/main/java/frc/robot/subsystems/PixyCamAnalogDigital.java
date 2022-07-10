package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The code for retrieving information from the PixyCam using analog/digital ports.
 * Be consistent about your usage of X vs Y, you can only send one or the other.
 * @author M. Francis
 */
public class PixyCamAnalogDigital extends SubsystemBase {

  //Ports
  private AnalogInput analog;
  private DigitalInput digital;

  //Debug mode
  private final boolean DEBUG = true;

  /**
   * Creates a PixyCamAnalog object
   * @param analogPort The analog port used for the PixyCam
   * @param digitalPort The digital port used for the PixyCam
   */
  public PixyCamAnalogDigital(int analogPort, int digitalPort){
    this.analog = new AnalogInput(analogPort);
    this.digital = new DigitalInput(digitalPort);
  }
  
  @Override
  public void periodic(){
    //Put on the SmartDashboard whether or not we see a target.
    SmartDashboard.putBoolean("Debug/Pixy/sees target", pixySeesTarget());

    if(DEBUG){
      //Put other information if we have DEBUG mode on.
      SmartDashboard.putNumber("Debug/Pixy/angle", getPixyTargetAngleX());
      SmartDashboard.putNumber("Debug/Pixy/x position", getPixyTargetX());
    }
  }

  /**
   * @return The voltage from the analog port as a raw value.
   * This is the largest target value.
   */
  private double getPixyVoltage(){
    return analog.getVoltage();
  }

  /**
   * @return Returns true if the PixyCam sees a target.
   */
  public boolean pixySeesTarget(){
    return digital.get();
  }

  /**
   * @return The angle in degrees of the largest target from the center of the
   * PixyCam. Use this method if the PixyCam is sending x information over the 
   * analog port. This will range from -30 to 30 as the horizontal field of
   * view of a Pixy2 is 60 degrees. Returns 0 if there is no target.
   */
  public double getPixyTargetAngleX(){
    if(!pixySeesTarget()) return 0;
    return ((getPixyVoltage() / 3.3) * 60) - 30;
  }
  /**
   * @return The angle in degrees of the largest target from the center of the
   * PixyCam. Use this method if the PixyCam is sending y information over the
   * analog port. This will range from -20 to 20 as the vertical field of
   * view of a Pixy2 is 60 degrees. Returns 0 if there is no target.
   */
  public double getPixyTargetAngleY(){
    if(!pixySeesTarget()) return 0;
    return ((getPixyVoltage() / 3.3) * 40) - 20;
  }

  /**
   * @return The position in pixels of the largest target from the left side of
   * the PixyCam. Use this method if the PixyCam is sending x information over
   * the analog port. This will range from 0 to 315. Returns -1 if no target
   * was found.
   */
  public int getPixyTargetX(){
    if(!pixySeesTarget()) return -1;
    return (int)((getPixyVoltage() / 3.3) * 315);
  }

  /**
   * @return The position in pixels of the largest target from the bottom side
   * of the PixyCam. Use this method if the PixyCam is sending y information
   * over the analog port. This will range from 0 to 207. Returns -1 if no
   * target was found.
   */
  public int getPixyTargetY(){
    if(!pixySeesTarget()) return -1;
    return (int)((getPixyVoltage() / 3.3) * 207);
  }
}
