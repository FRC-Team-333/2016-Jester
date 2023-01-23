package org.usfirst.frc.team333.robot;

import java.util.Map;
import java.util.TreeMap;

import org.usfirst.frc.team333.robot.RobotMap.AutoMode;
import org.usfirst.frc.team333.robot.RobotMap.TransMode;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotDashboard {
  private final String LABEL_GYRO_READING       = new String("Gyro Heading:");
  private final String LABEL_RPM_READING        = new String("Shooter RPM:");
  private final String LABEL_COUNT              = new String("Shooter Count:");
  private final String LABEL_JOYSTICK_Y         = new String("Joystick [Y]:");
  //private final String LABEL_COMPRESSOR_READING = new String("PSI:");
  //private final String LABEL_COMPRESSOR_SWITCH  = new String("Compressor:");
  private final String LABEL_TRANSMISSION_STATE = new String("Transmission:");
  private final String LABEL_GEAR_HOLDER_STATE  = new String("Gear Holder:");
  private final String LABEL_AUTO_MODE_CHOOSER  = new String("[Auto-Config] Auto Mode:");
  private final String LABEL_TRANS_MODE_CHOOSER = new String("[Teleop-Config] Transmission Mode:");

  protected RobotDashboard() {
    m_configValues = new TreeMap<String, Double>();
    m_autoChooser = new SendableChooser<AutoMode>();  
    m_autoChooser.addDefault(AutoMode.CENTER.name(), AutoMode.CENTER);
    m_autoChooser.addObject(AutoMode.LEFT_SIDE.name(), AutoMode.LEFT_SIDE);
    m_autoChooser.addObject(AutoMode.RIGHT_SIDE.name(), AutoMode.RIGHT_SIDE);
    SmartDashboard.putData(LABEL_AUTO_MODE_CHOOSER, m_autoChooser); 
    m_transChooser = new SendableChooser<TransMode>();        
    m_transChooser.addDefault(TransMode.AUTO.name(), TransMode.AUTO);
    m_transChooser.addObject(TransMode.MANUAL.name(), TransMode.MANUAL);
    SmartDashboard.putData(LABEL_TRANS_MODE_CHOOSER, m_transChooser); 
    m_monitor = new Monitor();
    Thread thread = new Thread(m_monitor);
    thread.start();
  }

  public static RobotDashboard getInstance() {
    if (instance == null) {
      instance = new RobotDashboard();
    }
    return instance;
  }

  public void setGyro(NavXGyro gyro) {
    m_monitor.setGyro(gyro);
  }

  public void setRpmCounter(RPMCounter rpmCounter) {
    m_monitor.setRpmCounter(rpmCounter);
  }
 
  public void setCompressor(Compressor compressor) {
    m_monitor.setCompressor(compressor);
  }
  
  
  public void setTransmission(Solenoidal transmission) {
    m_monitor.setTransmission(transmission);
  }
  
  public void setGearHolder(Solenoidal gearHolder) {
    m_monitor.setGearHolder(gearHolder);
  }
  
  public void setJoystick(Joystick joystick) {
    m_monitor.setJoystick(joystick);
  }

  public void setMonitoredValue(String key, double value) {
    m_monitor.setValue(key, value);
  }

  public void setConfigValue(String key, double value) {
    m_configValues.put(key, value);
    SmartDashboard.putNumber(key, value);
  }

  public double getConfigValue(String key) {
    if (m_configValues.containsKey(key)) {
      m_configValues.put(key, SmartDashboard.getNumber(key,0.0));
      return m_configValues.get(key);
    }
    return -1.0;
  }
  
  public AutoMode getAutoMode() {
    return m_autoChooser.getSelected();
  }
  
  public TransMode getTransMode() {
    return m_transChooser.getSelected();
  }

  class Monitor implements Runnable {
    public Monitor() {
      m_doubleValues = new TreeMap<String, Double>();
    }
    
    public void setJoystick(Joystick joystick) {
      m_joystick = joystick;
    }

    public void setGearHolder(Solenoidal gearHolder) {
      m_gearHolder = gearHolder;
    }

    public void setGyro(NavXGyro gyro) {
      m_gyro = gyro;
    }

    public void setRpmCounter(RPMCounter rpmCounter) {
      m_rpmCounter = rpmCounter;
    }
    
    public void setCompressor(Compressor compressor) {
      m_compressor = compressor;
    }
    

    public void setTransmission(Solenoidal transmission) {
      m_transmission = transmission;
    }

    public void setValue(String key, double value) {
      m_doubleValues.put(key, value);
    }

    @Override
    public void run() {
      SmartDashboard.putNumber(LABEL_GYRO_READING, -1);
      SmartDashboard.putNumber(LABEL_COUNT, -1);
      SmartDashboard.putNumber(LABEL_RPM_READING, -1);
      //SmartDashboard.putNumber(LABEL_COMPRESSOR_READING, -1);
      //SmartDashboard.putString(LABEL_COMPRESSOR_SWITCH, "Unknown");
      SmartDashboard.putString(LABEL_TRANSMISSION_STATE, "Unknown");
      SmartDashboard.putString(LABEL_GEAR_HOLDER_STATE, "Unknown");  

      while (true) {
        // Paint data for specific hardware
        if (m_gyro != null) {
          SmartDashboard.putNumber(LABEL_GYRO_READING, m_gyro.getAngle());
          //SmartDashboard.putNumber("Native Angle",m_gyro.getNativeAngle());
          //SmartDashboard.putNumber("Current Offset", m_gyro.getCurrentOffset());
        }
        if (m_rpmCounter != null) {
          SmartDashboard.putNumber(LABEL_RPM_READING, m_rpmCounter.getRpm());
          SmartDashboard.putNumber(LABEL_COUNT, m_rpmCounter.getRawCount());
        }
        
        if (m_compressor != null) {
          /*
          SmartDashboard.putString(LABEL_COMPRESSOR_SWITCH, 
                                   m_compressor.getPressureSwitchValue() ? "Stopped" : "Filling");
          SmartDashboard.putNumber(LABEL_COMPRESSOR_READING, m_compressor.getCompressorCurrent());
          */
        }
       
        if (m_transmission != null) {
          SmartDashboard.putString(LABEL_TRANSMISSION_STATE, m_transmission.get() ? "Low" : "High");
        }
        if (m_gearHolder != null) {
          SmartDashboard.putString(LABEL_GEAR_HOLDER_STATE, m_gearHolder.get() ? "In" : "Out");
        }
        
        if (m_joystick != null) {
          SmartDashboard.putNumber(LABEL_JOYSTICK_Y, m_joystick.getY());
        }

        // Paint config values
        for (Map.Entry<String, Double> entry : m_doubleValues.entrySet()) {
          SmartDashboard.putNumber(entry.getKey(), entry.getValue().doubleValue());
        }

        Timer.delay(0.25);
      }
    }

    private NavXGyro                m_gyro;
    private RPMCounter              m_rpmCounter;
    private TreeMap<String, Double> m_doubleValues;
    private Compressor              m_compressor;
    private Solenoidal              m_transmission;
    private Solenoidal              m_gearHolder;
  }

  private static RobotDashboard      instance = null;
  private Monitor                    m_monitor;
  private SendableChooser<AutoMode>  m_autoChooser;
  private SendableChooser<TransMode> m_transChooser;
  private TreeMap<String, Double>    m_configValues;
  private Joystick                   m_joystick;
}
