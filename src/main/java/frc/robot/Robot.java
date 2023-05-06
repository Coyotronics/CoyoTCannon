// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Compressor;

//SOLENOIDS EXAMPLE PROJECT
/**
 * This is a sample program showing the use of the solenoid classes during operator control. Three
 * buttons from a joystick will be used to control two solenoids: One button to control the position
 * of a single solenoid and the other two buttons to control a double solenoid. Single solenoids can
 * either be on or off, such that the air diverted through them goes through either one channel or
 * the other. Double solenoids have three states: Off, Forward, and Reverse. Forward and Reverse
 * divert the air through the two channels and correspond to the on and off of a single solenoid,
 * but a double solenoid can also be "off", where the solenoid will remain in its default power off
 * state. Additionally, double solenoids take up two channels on your PCM whereas single solenoids
 * only take a single channel.
 */
public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);

  //Declare and initialize Compressor
  Compressor _compressor = new Compressor(0, PneumaticsModuleType.CTREPCM); //0 should be correct for the module

  //Declare and initialize Double Solenoid 
  private final DoubleSolenoid m_doubleSolenoid =
      new DoubleSolenoid( PneumaticsModuleType.CTREPCM, 6, 7);

  public void teleopInit()
  {
    //m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    //WORKED
    //turns off the compressor when switched to teleop
    //_compressor.disable(); // this worked

    //DOESN'T WORK
    //switch the output on the double solenoid, tested one at a time, nothing happened
    //m_doubleSolenoid.set(DoubleSolenoid.Value.kOff);
    //m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  @Override
  public void teleopPeriodic() {
     //DOESN'T WORK
     //switch the output on the Double Solenoid using the X and Y buttons on the Xbox Controller
     
      if (m_controller.getLeftBumper()) {
          m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
      } else if (m_controller.getRightBumper()) {
          m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
      }else if(m_controller.getYButton())
      {
            m_doubleSolenoid.set(DoubleSolenoid.Value.kOff);
      }
      else if (m_controller.getXButton())
      {
        m_doubleSolenoid.toggle();
      }

      //WORKS
      //enable and disable the compressor using the A and B buttons on the Xbox Controller
      if (m_controller.getAButton()) {
          _compressor.enableDigital();
      } else if (m_controller.getBButton()) {
          _compressor.disable();
      }
  }
}
