// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//Primary Robot Functions

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;



import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  //Xbox Controller
  //default is port 0
  private final XboxController m_controller = new XboxController(0);  

  //Victor SPX 
  //deviceNumber matches the device ID shown in PhoenixTunerX
  VictorSPX Vspx1 = new VictorSPX(0); //left 
  VictorSPX Vspx2 = new VictorSPX(1); //left
  VictorSPX Vspx3 = new VictorSPX(3); //right
  VictorSPX Vspx4 = new VictorSPX(4); //right

  VictorSPX solenoid = new VictorSPX(5); 

  //Declare and initialize Compressor
  Compressor _compressor = new Compressor(0, PneumaticsModuleType.CTREPCM); //0 should be correct for the module

  //Declare and initialize Double Solenoid 
  //6 and 7 are set as the forward and reverse channels
  //correspond to the ports on the PCM
   private final DoubleSolenoid m_doubleSolenoid =
       new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);

  //SPARKMAX STUFF
  private static final int deviceID = 47;
  private CANSparkMax m_motor;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  double rotations = 0.0;


  //private CANSparkMax m_motor2;
      

  @Override
  public void robotInit() {

      //solenoid motor controller
      //m_motor2 = new CANSparkMax(40, MotorType.kBrushless);


      //SPARKMAX POSITION CONTROL LOOP EXAMPLE CODE
      // initialize motor
      m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);

      /**
       * The restoreFactoryDefaults method can be used to reset the configuration parameters
       * in the SPARK MAX to their factory default state. If no argument is passed, these
       * parameters will not persist between power cycles
       */
      m_motor.restoreFactoryDefaults();

      /**
       * In order to use PID functionality for a controller, a SparkMaxPIDController object
       * is constructed by calling the getPIDController() method on an existing
       * CANSparkMax object
       */
      m_pidController = m_motor.getPIDController();

      // Encoder object created to display position values
      m_encoder = m_motor.getEncoder();

      // PID coefficients
      kP = 0.1; 
      kI = 1e-4;
      kD = 1; 
      kIz = 0; 
      kFF = 0; 
      kMaxOutput = 1; 
      kMinOutput = -1;

      // set PID coefficients
      m_pidController.setP(kP);
      m_pidController.setI(kI);
      m_pidController.setD(kD);
      m_pidController.setIZone(kIz);
      m_pidController.setFF(kFF);
      m_pidController.setOutputRange(kMinOutput, kMaxOutput);

      // display PID coefficients on SmartDashboard
      SmartDashboard.putNumber("P Gain", kP);
      SmartDashboard.putNumber("I Gain", kI);
      SmartDashboard.putNumber("D Gain", kD);
      SmartDashboard.putNumber("I Zone", kIz);
      SmartDashboard.putNumber("Feed Forward", kFF);
      SmartDashboard.putNumber("Max Output", kMaxOutput);
      SmartDashboard.putNumber("Min Output", kMinOutput);
      SmartDashboard.putNumber("Set Rotations", 0);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {

    //doesn't look like it did anything as compared to how the wheels turned without it
    Vspx1.setInverted(false);
    Vspx2.setInverted(false);
    Vspx3.setInverted(false);
    Vspx4.setInverted(false);

    //compressor starts turned off
    _compressor.disable();  

    //value corresponding to the pivot motor
    rotations = 0.0;
  }

  /** This function is called periodically during teleoperated mode. */
  
 
  @Override
  public void teleopPeriodic() {

    //gets the Xbox controller values for: 
    // * .2 to make it slower 
    //left joystick on the controller
    double vert = m_controller.getRawAxis(1) * .2;
    //right joystick on the controller
    double horiz = m_controller.getRawAxis(4) * .2;

    //if we aren't moving the the bot forwards or back, then we can turn it
    //Left and Right Motion
    if(vert >= -0.09 && vert <= 0.09)
    {
      Vspx1.set(ControlMode.PercentOutput, horiz);//left
      Vspx2.set(ControlMode.PercentOutput, horiz);//left
      Vspx3.set(ControlMode.PercentOutput, horiz);//right
      Vspx4.set(ControlMode.PercentOutput, horiz);//right
    }

    //if we aren't turning the bot, we can move it forwards and back
    //Forward Backward Motion
    else if(horiz >= -0.09 && horiz <= 0.09)
    {
      Vspx1.set(ControlMode.PercentOutput, vert * -1);//left
      Vspx2.set(ControlMode.PercentOutput, vert * -1);//left
      Vspx3.set(ControlMode.PercentOutput, vert);//right
      Vspx4.set(ControlMode.PercentOutput, vert);//right
    }
    //WORKS
     //switch the output on the Double Solenoid using buttons on the Xbox Controller
     if (m_controller.getLeftTriggerAxis() > 0) {          //retract
      //m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
      solenoid.set(ControlMode.PercentOutput, 0);//lefty
    } 
    else if (m_controller.getRightTriggerAxis() > 0) {     //extend
      //m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
      //m_motor2.setVoltage(12);
      //m_motor2.set(1);
      solenoid.set(ControlMode.PercentOutput, 1);//left open

    }

    //WORKS
    //enable and disable the compressor using the X and Y buttons on the Xbox Controller
    if(m_controller.getYButton()){
          _compressor.disable();
    }
    else if (m_controller.getXButton()){
      _compressor.enableDigital();
    }

    //SPARKMAX MOTORCONTROLLER STUFF
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    //double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    //pivot motor rotates forwards and back using the buttons A and B
    if(m_controller.getAButton())
    {
      rotations +=1 ;
    }
    else if(m_controller.getBButton())
    {
      rotations -=1;
    }

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     *  com.revrobotics.CANSparkMax.ControlType.kPosition
     *  com.revrobotics.CANSparkMax.ControlType.kVelocity
     *  com.revrobotics.CANSparkMax.ControlType.kVoltage
     */
    m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("ProcessVariable", m_encoder.getPosition());

    // /**
    //  * There are several useful bus measurements you can get from the SparkMax.
    //  * This includes bus voltage (V), output current (A), Applied Output 
    //  * (duty cycle), and motor temperature (C)
    //  */
    // double busVoltage = m_motor2.getBusVoltage();
    // double current = m_motor2.getOutputCurrent();
    // double appliedOut = m_motor2.getAppliedOutput();
    // double temperature = m_motor2.getMotorTemperature();

    // // Open SmartDashboard when your program is running to see the values
    // SmartDashboard.putNumber("Bus Voltage", busVoltage);
    // SmartDashboard.putNumber("Current", current);
    // SmartDashboard.putNumber("Applied Output", appliedOut);
    // SmartDashboard.putNumber("Motor Temperature", temperature);

    // m_motor2.set(m_controller.getRawAxis(1));


  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
