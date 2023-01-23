package org.usfirst.frc.team333.robot;

import java.util.ArrayDeque;
import java.util.ArrayList;

import org.usfirst.frc.team333.robot.RobotMap.TransMode;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotUtils
{

  public static boolean dbl_equals_power(double a, double b, int precision) {
    return Math.abs(a - b) <= Math.pow(10, -precision);
  }

  public static boolean dbl_equals(double a, double b, double precision) {
    return Math.abs(a - b) <= precision;
  }
}


class FlashLight
{
  // With the Low Voltage, don't need to wait, it doesn't cycle.
  private static long BICYCLE_LIGHT_CYCLE_RESET_TIME_MS = 500;
  //private static long STANDARD_LIGHT_CYCLE_RESET_TIME_MS = 4500;
  private static int POST_CYCLE_TARGET = 2;
  private static long CYCLE_DELAY_MS = 1000;
  private boolean m_isOn, m_isForcedOn;
  private int     m_postCycleCount;
  private Relay   m_relay;
  private long    m_lastTurnedOff;
  private Relay.Value ON = Relay.Value.kForward; // Relay.Value.kForward;

  public FlashLight(int port)
  {
    m_relay = new Relay(port);
    m_isOn = false;
    m_lastTurnedOff = 0;
    m_postCycleCount = 0;
  }

  public void turnOff() {
    if (m_isOn && !m_isForcedOn) {
      m_relay.set(Relay.Value.kOff);
      if (m_postCycleCount < POST_CYCLE_TARGET) {
        if (m_lastTurnedOff - System.currentTimeMillis() < CYCLE_DELAY_MS * (m_postCycleCount +1 )) {
          m_postCycleCount++;
          m_relay.set(ON);
        }
      } else {
        m_isOn = false;
        m_lastTurnedOff = System.currentTimeMillis();
      }
    }
  }

  public void turnOn() {
    if (!m_isOn && (System.currentTimeMillis()
        - m_lastTurnedOff) > BICYCLE_LIGHT_CYCLE_RESET_TIME_MS) {
      m_isOn = true;
      m_postCycleCount = 0;
      m_relay.set(ON);
    }
  }

  public void forceTurnOn() {
    if (!m_isOn) {
      m_isOn = true;
      m_isForcedOn = true;
      m_relay.set(ON);
    }
  }

  public void revertForceOn() {
    if (m_isForcedOn) {
      m_isOn = false;
      m_isForcedOn = false;
      m_relay.set(Relay.Value.kOff);
    }
  }
}

class Solenoidal
{
  // Solenoids always come in pairs that are inverses of each other.
  // A "Solenoidal" is a wrapper around the solenoids to represent the two as
  // a single entity.
  private Solenoid m_solenoid1, m_solenoid2;
  private boolean  m_state;

  public Solenoidal(int id1, int id2)
  {
    m_solenoid1 = new Solenoid(id1);
    m_solenoid2 = new Solenoid(id2);
  }

  public void set(boolean value) {
    m_state = value;
    m_solenoid1.set(m_state);
    m_solenoid2.set(!m_state);
  }

  public boolean get() {
    return m_state;
  }
}

class SolenoidalPair
{
  private Solenoidal m_solenoidal1, m_solenoidal2;

  public SolenoidalPair(Solenoidal sol1, Solenoidal sol2)
  {
    m_solenoidal1 = sol1;
    m_solenoidal2 = sol2;
  }

  public void set(boolean value) {
    m_solenoidal1.set(value);
    m_solenoidal2.set(value);
  }
}

class VictorPair
{
  private Victor m_motor1, m_motor2;
  private double m_multiplier;

  public VictorPair(int id1, int id2, boolean inverted)
  {
    m_motor1 = new Victor(id1);
    m_motor2 = new Victor(id2);
    m_multiplier = inverted ? -1.0 : 1.0;
  }

  public void set(double value) {
    m_motor1.set(value);
    m_motor2.set(value * m_multiplier);
  }
}

class NavXGyro
{
  AHRS   m_ahrs;
  double m_lastResetAngle = 0.0;

  public NavXGyro(SPI.Port port)
  {
    m_ahrs = new AHRS(port);
    m_ahrs.reset();
    m_ahrs.resetDisplacement();
    do {
      m_lastResetAngle = m_ahrs.getAngle();
    } while (m_lastResetAngle == 0.0);
  }

  public void reset() {

    // m_ahrs.reset(); m_ahrs.resetDisplacement();

    do {
      m_lastResetAngle = m_ahrs.getAngle();
    } while (m_lastResetAngle == 0.0);
    System.out.println("LastResetAngle = " + m_lastResetAngle);
  }

  public double getAngle() {
    return m_ahrs.getAngle() - m_lastResetAngle;
  }

  public double getNativeAngle() {
    return m_ahrs.getAngle();
  }

  public double getCurrentOffset() {
    return m_lastResetAngle;
  }
}

class UltrasonicMonitor implements Runnable
{
  Ultrasonic m_ultrasonic;

  public UltrasonicMonitor(Ultrasonic ultrasonic)
  {
    m_ultrasonic = ultrasonic;
  }

  @Override
  public void run() {
    while (m_ultrasonic != null) {
      SmartDashboard.putNumber("Ultrasonic", m_ultrasonic.getRangeInches());
      Timer.delay(0.05);
    }
  }
}

class PotentiometerMonitor implements Runnable
{
  AnalogPotentiometer m_potentiometer;

  public PotentiometerMonitor(AnalogPotentiometer potentiometer)
  {
    m_potentiometer = potentiometer;
  }

  @Override
  public void run() {
    while (m_potentiometer != null) {
      SmartDashboard.putNumber("Potentiometer", m_potentiometer.get());
      Timer.delay(0.05);
    }
  }
}

class DummyAHRS
{
  public DummyAHRS(Port port)
  {
  }

  public void reset() {
  }

  public void resetDisplacement() {
  }

  public double getAngle() {
    return 123.45;
  }
}

class RPMMotor implements PIDOutput
{
  public RPMMotor(Victor motor)
  {
    m_motor = motor;
  }

  private Victor m_motor;

  @Override
  public void pidWrite(double arg0) {
    System.out.println("pidWrite: " + arg0);
    m_motor.pidWrite(arg0);
  }
}

class RPMCounter implements PIDSource
{
  public RPMCounter(DigitalInput rpmSensor)
  {
    m_counter = new Counter(rpmSensor);
    m_counter.setMaxPeriod(.1);
    m_counter.setUpdateWhenEmpty(true);
    m_counter.setReverseDirection(false);
    m_counter.setSamplesToAverage(10);
    m_counter.setDistancePerPulse(60);
    m_timer = new Timer();
    m_timer.start();
    m_history = new ArrayList<Double>();
  }

  public void reset() {
    m_timer.reset();
    m_counter.reset();
  }

  public double getRpm() {
    return m_counter.getRate();
  }

  public int getRawCount() {
    return m_counter.get();
  }

  @Override
  public PIDSourceType getPIDSourceType() {
    return m_pidSourceType;
  }

  @Override
  public double pidGet() {
    double rpm = getRpm();
    if (rpm <= MAX_RANGE) {
      appendToHistory(rpm);
    }

    rpm = averageHistory();

    System.out.println("PidGet = " + rpm);
    return rpm;
  }

  @Override
  public void setPIDSourceType(PIDSourceType pidSourceType) {
    m_pidSourceType = pidSourceType;
  }

  private double averageHistory() {
    double sum = 0.0;
    for (int i = 0; i < m_history.size(); i++) {
      sum += m_history.get(i);
    }
    return sum / m_history.size();
  }

  private void appendToHistory(double data) {
    m_history.add(data);
    if (m_history.size() > HISTORY_LIMIT) {
      m_history.remove(0);
    }
  }

  private ArrayList<Double> m_history;
  private static double     HISTORY_LIMIT   = 20;
  private Counter           m_counter;
  private Timer             m_timer;
  private PIDSourceType     m_pidSourceType = PIDSourceType.kDisplacement;
  public static double      MAX_RANGE       = 4800.0;
}

class MultiSpeedController implements SpeedController
{

  private SpeedController[] m_speedControllers;
  private double            m_speed;
  private boolean           m_inverted;

  public MultiSpeedController(SpeedController... speedControllers)
  {
    m_speedControllers = speedControllers;
    set(0);
  }

  @Override
  public double get() {
    return m_speed;
  }

  @Override
  public void set(double speed) {
    m_speed = speed;

    for (SpeedController speedController : m_speedControllers) {
      speedController.set(m_speed);
    }
  }

  @Override
  public void pidWrite(double output) {
    set(output);
  }

  @Override
  public void disable() {
    for (SpeedController speedController : m_speedControllers) {
      speedController.disable();
    }
  }

  @Override
  public boolean getInverted() {
    return m_inverted;
  }

  @Override
  public void setInverted(boolean inverted) {
    m_inverted = inverted;
    for (SpeedController speedController : m_speedControllers) {
      speedController.setInverted(m_inverted);
    }
  }

  @Override
  public void stopMotor() {
    m_speed = 0.0;

    for (SpeedController speedController : m_speedControllers) {
      speedController.stopMotor();
    }
  }
}

// This class is a significantly simplified version of a PID controller,
// that re-evalutes only at the time of "setSetpoint" rather than in a
// background thread.
class Regulator
{
  // Take the proportional difference between what the speed is right now
  // (averaged over the last 20-some-odd samples) and where it needs to be,
  // compared with the current Power Input to the motor and its total range,
  // and adjust the output power accordingly.
}

// This is wrapper class around RobotDrive that acts as an automatic
// transmission.
// It sets thresholds beyond which it'll shift.
// The logic hierarchy is as follows:
// 1) if we're powering at below LOW_THRESHOLD, always shift into low gear.
// 2) if we're powering above HIGH_THRESHOLD, and we have been for the last
// half-second, switch into HIGH.
class AutoTransDrive
{
  public static final int HISTORY_LENGTH_MS = 250;
  public static final double HIGH_THRESHOLD = 0.99;
  public static final double LOW_THRESHOLD  = 0.40;

  public AutoTransDrive(SpeedController leftDrive, SpeedController rightDrive, Solenoidal transmission)
  {
    m_transmission = transmission;
    m_drive = new RobotDrive(leftDrive, rightDrive);
    m_drive.setExpiration(0.1);
    m_drive.setSafetyEnabled(true);
    m_drive.setMaxOutput(1.0);
    m_history = new History(HISTORY_LENGTH_MS, true);
    m_transToggled = false;
  }
  
  public void arcadeDrive(Joystick stick)
  {
    m_drive.arcadeDrive(stick);

    // Even if Auto isn't on, we should be keeping track of the history so we can turn it on later.
    m_history.appendToHistory(stick.getY());

    TransMode mode = RobotDashboard.getInstance().getTransMode();
    if (mode != TransMode.AUTO) {
      // Transmission (toggle)
      if (stick.getRawButton(5)) {
        // Only toggle once per "press". Only re-toggle when released.
        if (!m_transToggled) {
          m_transmission.set(!m_transmission.get());
          m_transToggled = true;
        }
      } else {
        m_transToggled = false;
      }
      
      return;
    }

    if (stick.getTrigger()) {
      lowTransmission(); 
      return;
    }
    
    double effectiveSpeed = m_history.getHistoryAverage();
    
    if (effectiveSpeed < LOW_THRESHOLD) {
      if (lowTransmission()) {
        System.out.println("Auto-Switching to Low Transmission");
      }
      return;
    }
    if (effectiveSpeed >= HIGH_THRESHOLD) {
      if (highTransmission()) {
        System.out.println("Auto-Switching to High Transmission");
      }
      return;
    }
    
  }
  public void setLeftRightMotorOutputs(double left, double right) {
    if (m_drive != null) {
      m_drive.setLeftRightMotorOutputs(left, right);
    }
  }
  
  public void arcadeDrive(double speed, double angle) {
    if (m_drive != null) {
      if (speed == 0.0 && angle == 0.0) {
        m_drive.stopMotor();
      } else {
        m_drive.arcadeDrive(speed, angle);
      }
    }
  }
  
  public boolean lowTransmission() {
    if (m_transmission == null) {
      DriverStation.reportError("Transmission called, but not instantiated\n",
          false);
      return false;
    }
    boolean changed = !m_transmission.get();
    m_transmission.set(true);
    return changed;
  }
  
  public boolean highTransmission() {
    if (m_transmission == null) {
      DriverStation.reportError("Transmission called, but not instantiated\n",
          false);
      return false;
    }
    boolean changed = m_transmission.get();
    m_transmission.set(false);
    return changed;
  }

  private History               m_history;
  private RobotDrive            m_drive;
  private Solenoidal            m_transmission;
  private boolean               m_transToggled;
}

class History
{
  History(int historyLenMs, boolean useAbsValue) {
    m_historyLenMs = historyLenMs;
    m_history = new ArrayDeque<TimeEntry>();
    m_useAbsValue = useAbsValue;
  }

  public void appendToHistory(double magnitude) {
    long now = System.currentTimeMillis();
    while (m_history.peekFirst() != null && (now - m_history.peekFirst().time) > m_historyLenMs) {
      m_history.pollFirst();
    }
    m_history.addLast(new TimeEntry(now, m_useAbsValue ? Math.abs(magnitude) : magnitude));
  }
  
  public double getHistoryAverage() {
    double sum = 0.0;
    if (m_history.size() < 1) { return sum; }
    for (TimeEntry e : m_history) {
      sum += e.magnitude;
    }
    //System.out.println("Average = "+(sum/m_history.size())+" ; Size="+m_history.size());
    return sum / m_history.size();
  }
  
  private class TimeEntry
  {
    public TimeEntry(long time_, double magnitude_)
    {
      time = time_;
      magnitude = magnitude_;
    }
  
    public long   time;
    public double magnitude;
  }

  private ArrayDeque<TimeEntry> m_history;
  private int                   m_historyLenMs;
  private boolean               m_useAbsValue;
}
