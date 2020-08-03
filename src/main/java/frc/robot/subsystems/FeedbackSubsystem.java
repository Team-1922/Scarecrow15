/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.ctre.phoenix.CANifier;



import frc.robot.Constants;

public class FeedbackSubsystem extends SubsystemBase {

  private CANifier m_LED  = new CANifier(Constants.kCanifier0);
  boolean m_LEDsOn = false;
  
  // set up the limelight
  


  /**
   * Creates a new DriveSystem.
   */
  public FeedbackSubsystem() {
    super();
     
    // setLED (255,191,0);
    register(); // tell the command scheduler about this subsystem so that periodic is called
    
  }

  public void enableLED(boolean turnOn){
    if(!m_LEDsOn && (turnOn == true)) {
      setLED(Constants.kLEDOzRamRed, Constants.kLEDOzRamGreen, Constants.kLEDOzRamBlue);
      m_LEDsOn = true;
    }
    else if (m_LEDsOn && (turnOn == false)) {
      setLED(0,0,0);
      m_LEDsOn = false;
    }
  }

  public boolean ledsOn() {
    return m_LEDsOn;
  }

  

  public void setLED (float r, float g, float b){
    
    // green, red, blue
    float _rgb[] = new float[3];
    _rgb[0] = g;
    _rgb[1] = r;
    _rgb[2] = b;
    m_LED.setLEDOutput(_rgb[0], CANifier.LEDChannel.LEDChannelA);  // green
		m_LED.setLEDOutput(_rgb[1], CANifier.LEDChannel.LEDChannelB); // red
		m_LED.setLEDOutput(_rgb[2], CANifier.LEDChannel.LEDChannelC);   // blue
   
  }
  
  @Override
  public void periodic() {

  }
}
