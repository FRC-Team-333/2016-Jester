package org.usfirst.frc.team333.robot;

import java.util.ArrayList;

import org.usfirst.frc.team333.robot.RobotMap.DigitalSensorPort;
import org.usfirst.frc.team333.robot.RobotMap.VictorPort;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;

public class RobotShooter
{
  private static double INTAKE_SPEED   = 1.0;
  private static final String LABEL_MANUAL_OVERRIDE = "Manual Shooter Speed: ";
  /*
  private static final String LABEL_SHOOTER_TARGET_RPM = "Shooter RPM Target:";
  private static final String LABEL_P = "P:";  
  private static final String LABEL_I = "I:";
  private static final String LABEL_D = "D:";
  private static final double DEFAULT_SHOOTER_TARGET_RPM = 1000;
  */
  private static final double DEFAULT_MANUAL_OVERRIDE = 0.48;
  /*
  private static final double DEFAULT_P = 0.3;//0.1;
  private static final double DEFAULT_I = 0.001;// 0.001;
  private static final double DEFAULT_D = 0.01;//0.00;
  */

  public RobotShooter()
  {
    /* Configure the standard RobotDrive Chassis */
    try {
      m_shooterMotor = new Victor(VictorPort.SHOOTER_MOTOR);
    } catch (Exception ex) {
      DriverStation.reportError("Could not instantiate Shooter Motor", false);
    }
    
    try {
    	m_intakeMotor = new Victor(VictorPort.INTAKE_MOTOR);
    } catch (Exception ex) {
    	DriverStation.reportError("Could not instantiate Intake Motor", false);
    }
    
    try {
      m_rpmReadings = new ArrayList<Double>();
      m_rpmSensor = new DigitalInput(DigitalSensorPort.SHOOTER_RPM_SENSOR);
      m_rpmCounter = new RPMCounter(m_rpmSensor);
      RobotDashboard.getInstance().setRpmCounter(m_rpmCounter);
    } catch (Exception ex) {
      DriverStation.reportError("Could not instantiate RPM Gauge/Counter",
          false);
    }
    
    /*
    m_motorPIDController = new PIDController(DEFAULT_P, DEFAULT_I, DEFAULT_D, m_rpmCounter, new RPMMotor(m_shooterMotor));
    m_motorPIDController.setInputRange(0, RPMCounter.MAX_RANGE);
    m_motorPIDController.setOutputRange(0.5, 0.8);
    */

    m_isSpinningUp = false;
    /*
    RobotDashboard.getInstance().setConfigValue(LABEL_SHOOTER_TARGET_RPM,
                                                DEFAULT_SHOOTER_TARGET_RPM);
    RobotDashboard.getInstance().setConfigValue(LABEL_P, DEFAULT_P);
    RobotDashboard.getInstance().setConfigValue(LABEL_I, DEFAULT_I);
    RobotDashboard.getInstance().setConfigValue(LABEL_D, DEFAULT_D);
    */
    RobotDashboard.getInstance().setConfigValue(LABEL_MANUAL_OVERRIDE, DEFAULT_MANUAL_OVERRIDE);
  }
  

  public void periodic(Joystick joystick) {
    if (joystick == null) {
      DriverStation.reportError("Could not run Shooter periodic, no joystick",
          false);
      return;
    }
    
    // Shooter
    if (joystick.getTrigger()) {
      if (m_shooterMotor != null) {
        m_shooterMotor.set(RobotDashboard.getInstance().getConfigValue(LABEL_MANUAL_OVERRIDE));
      }
    } else {
      if (m_shooterMotor != null) {
        m_shooterMotor.set(0.0);
      }
    }
    
    // Intake
    if (joystick.getRawButton(2)) {
    	if (m_intakeMotor != null) {
    		m_intakeMotor.set(INTAKE_SPEED);
    	} 
    } else {
    	if (m_intakeMotor != null) {
    		m_intakeMotor.set(0);
    	}
    }
    
    
    /*
    if (joystick.getRawButton(11)) {
      if (m_motorPIDController != null) {        
        if(!m_motorPIDController.isEnabled()) {
          // Whenever we go from the trigger being not-held to being held
          //  (i.e., when it's first pressed down), we should reset the counter.
          System.out.println("Trigger pressed");
          m_rpmCounter.reset();
          
          double p = RobotDashboard.getInstance().getConfigValue(LABEL_P);      
          double i = RobotDashboard.getInstance().getConfigValue(LABEL_I);     
          double d = RobotDashboard.getInstance().getConfigValue(LABEL_D);
          m_motorPIDController.setPID(p, i, d);
          
          m_motorPIDController.enable();
          m_shooterSetpoint = RobotDashboard.getInstance().getConfigValue(LABEL_SHOOTER_TARGET_RPM);
          System.out.println("PID Controller Enabled, setting rate at "+m_shooterSetpoint);
        }
        
        m_motorPIDController.setSetpoint(m_shooterSetpoint);
      }
    } else {
      if (m_motorPIDController != null) {
        if (m_motorPIDController.isEnabled()) {
          System.out.println("Trigger released");
          m_motorPIDController.disable();
        }
      }
    }
    */
  
    
  }
  
  public boolean isSpinningUp() {
	  if (m_rpmSensor == null) {return false;}
	  
	  if (m_targetSpeed == null) {
		m_rpmReadings.clear();
		m_isSpinningUp = false;
		return false;
	  }
	  
	  if (!m_isSpinningUp) {
		  m_rpmCounter.reset();
		  m_isSpinningUp = true;
	  }
	  /*
	  double currentSpeed = m_rpmCounter.get(),
	      targetSpeed = m_targetSpeed.doubleValue();
	      
	  RobotDashboard.getInstance().setMonitoredValue("Target RPM", targetSpeed);
	  m_rpmReadings.add(new Double(currentSpeed));
	  */
	  return false;
  }
  
  public void target(double value) {
	  m_targetSpeed = new Double(value);
  }
 
  private RPMCounter        m_rpmCounter;
  private Victor            m_shooterMotor;
  //private PIDController     m_motorPIDController;
  private Victor            m_intakeMotor;
  private Double            m_targetSpeed;
  private DigitalInput      m_rpmSensor;
  private ArrayList<Double> m_rpmReadings;
  //private double            m_shooterSetpoint;
  private boolean           m_isSpinningUp;
}
