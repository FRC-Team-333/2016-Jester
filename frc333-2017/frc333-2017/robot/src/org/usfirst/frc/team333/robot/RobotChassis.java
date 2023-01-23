package org.usfirst.frc.team333.robot;

import java.util.ArrayList;

import org.usfirst.frc.team333.robot.RobotMap.SolenoidPort;
import org.usfirst.frc.team333.robot.RobotMap.VictorPort;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;

public class RobotChassis
{

  public static final int    AVERAGING_QUEUE_SIZE    = 10;
  public static final double ANGLE_COMPARE_PRECISION = 1.0;
  public static final double ANGLING_DRIVE_THRESHOLD = 10.0;  
  private static double CLIMBER_SPEED  = -1.0;
  private static double GEAR_TOGGLE_INTERVAL_MS = 1000;

  
  public RobotChassis()
  {
    /* Instantiate the compressor */
    try {
      m_compressor = new Compressor(RobotMap.CompressorPort.MAIN_COMPRESSOR);
      RobotDashboard.getInstance().setCompressor(m_compressor);
    } catch (Exception ex) {
      DriverStation.reportError("Could not start Compressor\n", false);
    }
    
    /* Initiate the RobotDrive Chassis and Tranmission Solenoids*/
    try {
      Solenoidal transmission = new Solenoidal(SolenoidPort.DRIVE_TRANS_LOW,
          SolenoidPort.DRIVE_TRANS_HIGH);
      RobotDashboard.getInstance().setTransmission(m_transmission);

      SpeedController leftDrive = new MultiSpeedController(new Victor(VictorPort.DRIVE_LEFT_1),
                                                           new Victor(VictorPort.DRIVE_LEFT_2),
                                                           new Victor(VictorPort.DRIVE_LEFT_3));
      SpeedController rightDrive = new MultiSpeedController(new Victor(VictorPort.DRIVE_RIGHT_1),
                                                            new Victor(VictorPort.DRIVE_RIGHT_2),
                                                            new Victor(VictorPort.DRIVE_RIGHT_3));
      m_drive = new AutoTransDrive(leftDrive, rightDrive, transmission);
    } catch (Exception ex) {
      DriverStation.reportError("Could not start DriveTrain/Transmission\n", false);
    }
    
   
   /* Instantiate the Gear Solenoids */
   try {
     m_gearHolder = new Solenoidal(SolenoidPort.GEAR_HOLDER_PISTON_IN, SolenoidPort.GEAR_HOLDER_PISTON_OUT);
     RobotDashboard.getInstance().setGearHolder(m_gearHolder);
   } catch (Exception ex) {
     DriverStation.reportError("Could not initialize Gear Holder\n", false);
   }
  
   /* Instantiate the NavX */
    try {
      // TODO: For some reason, this doesn't throw gracefully, and causes
    // everything to crash if it's not connected.
    //
       //m_gyro_readings = new ArrayList<Double>();
       m_gyro = new NavXGyro(SPI.Port.kMXP);
       m_gyro.reset();
       RobotDashboard.getInstance().setGyro(m_gyro);
    } catch (Exception ex) {
       DriverStation.reportError("Could not initialize NavX\n", false);
    }
   
    /* Instantiate the Climber Motor */
    try {
      m_climberMotor = new Victor(VictorPort.CLIMBER_MOTOR);
    } catch (Exception ex) {
      DriverStation.reportError("Could not instantiate Climber Motor", false);
    }

    /* Instantiate the Flashlight */
    try {
      m_light = new FlashLight(RobotMap.RelayPort.LIGHT_RELAY);
    } catch (Exception ex) {
      DriverStation.reportError("Could not instantiate Flashlight Motor", false);
    }
    
    m_gearHolderLastToggled = 0;
    //m_hasStartedAutoAngling = false;
    System.out.println("Chassis initialized.");
  } 

  public void highTransmission() {
    m_drive.highTransmission();
  }

  public void lowTransmission() {
    m_drive.lowTransmission();
  }
  
  public void gearIn() {
    if (m_gearHolder == null) {
      DriverStation.reportError("Gear Holder called, but not instantiated\n", false);
      return;
    }
    
    /*
    // Begin: Gear holder toggle delay for Heavy GearHolder
    if (m_gearHolderLastToggled - System.currentTimeMillis() < GEAR_TOGGLE_INTERVAL_MS) {
      return;
    }
    m_gearHolderLastToggled = System.currentTimeMillis();
    // End: Gear holder toggle delay for Heavy GearHolder
    */
    
    m_gearHolder.set(true);
    if (m_light != null) {
      m_light.turnOff();
    }
  }
  
  public void gearOut() {
    if (m_gearHolder == null) {
      DriverStation.reportError("Gear Holder called , but not instantiated", false);
      return;
    }
    
    /*
    // Begin: Gear holder toggle delay for Heavy GearHolder
    if (m_gearHolderLastToggled - System.currentTimeMillis() < GEAR_TOGGLE_INTERVAL_MS) {
      return;
    }
    m_gearHolderLastToggled = System.currentTimeMillis();
    // End: Gear holder toggle delay for Heavy GearHolder
    */
    
    m_gearHolder.set(false);
    if (m_light != null) {
      m_light.turnOn();
    }
  }

  public void periodic(Joystick joystick) {
    if (joystick == null) {
      DriverStation.reportError("No joystick, cannot run Chassis periodic\n", false);
      return;
    }

    if (m_drive != null) {
      m_drive.arcadeDrive(joystick); // m_drive with arcade style
    }

    if (m_compressor != null) {
      m_compressor.setClosedLoopControl(true);
    }
    
    // Gear tray (hold)
    if (joystick.getRawButton(4)) {
      gearOut();
    } else {
      gearIn();
    }

    /*
    // Gear tray (toggle)
    if (joystick.getRawButton(4)) {
      // Only toggle once per "press". Only re-toggle when released.
      if (!m_gearToggled) {
        m_gearHolder.set(!m_gearHolder.get());
        m_gearToggled = true;
      }
    } else {
      m_gearToggled = false;
    }
    */

    if (joystick.getRawButton(3)) {
      if (m_climberMotor != null) {
        m_climberMotor.set(CLIMBER_SPEED);
      }
    } else {
      if (m_climberMotor != null) {
        m_climberMotor.set(0.0);
      }
    }
  }

  public void drive(double left, double right) {
    if (m_drive != null) {
      m_drive.setLeftRightMotorOutputs(left, right);
    }
  }

  public void arcadeDrive(double speed, double angle) {
    if (m_drive != null) {
      m_drive.arcadeDrive(speed, angle);
    }
  }

  
  /*
   * This function is intended to be called repeatedly (dozens of times per
   * second). It checks to see whether there is a particular "Target Angle" we
   * are trying to turn towards, and then turns to that angle until it's facing
   * that way. Every time it's called, it sees where it is right now vs. where
   * it wants to be, and continues to instruct the motors to move as they need
   * to.
   */

  public boolean isAutoAngling() { // we talking about the gyro
    /*
    if (m_gyro == null) { return false; }

    if (m_chassisTargetAngle == null) {
      m_drive.drive(0.0, 0.0);
      m_gyro_readings.clear();
      m_hasStartedAutoAngling = false;
      return false;
    }

    // We have to reset the gyro before we start doing any move.
    if (!m_hasStartedAutoAngling) {
      m_gyro.reset();
      m_hasStartedAutoAngling = true;
    }

    double currentAngle = m_gyro.getAngle(),
        targetAngle = m_chassisTargetAngle.doubleValue();

    // Allow tolerance of a full degree, for now. We can up the precision later
    // if needed.
    RobotDashboard.getInstance().setMonitoredValue("Chassis Target Angle",
        targetAngle);
    m_gyro_readings.add(new Double(currentAngle));
    if (m_gyro_readings.size() < AVERAGING_QUEUE_SIZE) { return true; }
    m_gyro_readings.remove(0);

    double angle = 0.0;
    
    for (int i = 0; i < m_gyro_readings.size(); i++) {
      angle += m_gyro_readings.get(i).doubleValue();
    }
    currentAngle = angle / m_gyro_readings.size();

    // If we've reached our target, stop.
    if (Math.abs(currentAngle - targetAngle) < ANGLE_COMPARE_PRECISION) {
      m_chassisTargetAngle = null;
      m_drive.drive(0.0, 0.0);
      return false;
    }

    // If we're not there yet, continue to try to approach the target.
    double motor_power = (currentAngle > targetAngle ? 1.0 : -1.0);

    // Depending on how close we are, we should decrease our motor power.
    // This is the ghetto version of a PID Subsystem.
    if (Math.abs(currentAngle - targetAngle) < ANGLING_DRIVE_THRESHOLD) {
      motor_power *= (Math.abs(currentAngle - targetAngle)
          / ANGLING_DRIVE_THRESHOLD);
    }

    // Turn in place by pushing back on one motor, & forward on the other.
    m_drive.tankDrive(motor_power, -motor_power);
    */
    return true;
  }

  /*
  public void autoMove(double value) {
    m_chassisTargetAngle = new Double(value);
    isAutoAngling();
  }
  */

  public void resetGyro() {
    if (m_gyro == null) { return; }
    m_gyro.reset();
  }

  public double getGyroAngle() {
    if (m_gyro == null) { return -1.0; }
    return m_gyro.getAngle();
  }

  private NavXGyro          m_gyro;
  private AutoTransDrive    m_drive;
  private Victor            m_climberMotor;
  private Compressor        m_compressor;
  private Solenoidal        m_transmission;
  private boolean           m_transToggled;
  private long              m_gearHolderLastToggled;
  private Solenoidal        m_gearHolder;
  //private Double            m_chassisTargetAngle;
  //private boolean           m_hasStartedAutoAngling;
  //private ArrayList<Double> m_gyro_readings;
  private FlashLight        m_light;
}
