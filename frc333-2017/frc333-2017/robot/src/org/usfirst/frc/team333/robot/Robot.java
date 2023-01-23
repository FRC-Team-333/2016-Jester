
package org.usfirst.frc.team333.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;

import org.usfirst.frc.team333.robot.RobotMap.AutoMode;
import org.usfirst.frc.team333.robot.RobotMap.CameraPort;
import org.usfirst.frc.team333.robot.RobotMap.JoystickPort;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;

public class Robot extends IterativeRobot {
  
  public static final double AUTODRIVE_INITIAL_SPEED = 0.85;
  public static final double AUTODRIVE_FINAL_SPEED = 0.70;
  public static final double AUTODRIVE_TURN_POWER = 0.3; // Lower to ensure more accurate stop-point
  public static final double AUTODRIVE_TURN_SANITY_TIMER = 15.0;
  public static final double DEFAULT_AUTO_SIDE_INITIAL_DISTANCE = 90.0;
  public static final double DEFAULT_AUTO_SIDE_LEFT_ANGLE = 45.0;
  public static final double DEFAULT_AUTO_SIDE_RIGHT_ANGLE = 53.0;
  public static final double DEFAULT_AUTO_SIDE_FINAL_DISTANCE = 100.0;
  public static final double DEFAULT_AUTO_CENTER_DISTANCE = 86;
  public static final double DEFAULT_GYRO_MULTIPLIER = 0.21;
  public static final double AUTODRIVE_GEAROUT_DELAY = 0.5;
  public static final String LABEL_AUTO_SIDE_INITIAL_DISTANCE = "[Auto-Config] Side Initial Distance (in) ="; 
  public static final String LABEL_AUTO_SIDE_LEFT_ANGLE = "[Auto-Config] Left Side Turn Angle (deg) =";
  public static final String LABEL_AUTO_SIDE_RIGHT_ANGLE = "[Auto-Config] Right Side Turn Angle (deg) =";
  public static final String LABEL_AUTO_SIDE_FINAL_DISTANCE = "[Auto-Config] Side Final Distance (in) =";
  public static final String LABEL_AUTO_CENTER_DISTANCE = "[Auto-Config] Center Distance (in) =";
  public static final String LABEL_GYRO_MULTIPLIER = "[Auto-Config] Gyro Multiplier =";  
  
  /**
   * This function is called when the robot starts
   */
  @Override
  public void robotInit() {
    /* Logging */
    System.out.println("Calling RobotInit");
    
    /* Camera */
    try {
      m_simpleCameraServer = CameraServer.getInstance();
    } catch (Exception ex) {
      DriverStation.reportError("Cannot instantiate CameraServer", false);
    }
    
    if (m_simpleCameraServer != null) {
      // Front camera
      try { 
        UsbCamera camera = m_simpleCameraServer.startAutomaticCapture(CameraPort.FRONT_CAMERA);
        //camera.setResolution(320, 240);// 160,120
        camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 240, 180, 20);

        //camera.setResolution(160,120);
        System.out.println("Front Camera valid? - " + camera.isValid());
        System.out.println("Front Camera connected? - " + camera.isConnected());
      } catch (Exception ex) {
        DriverStation.reportError("Cannot start Front CameraServer\n", false);
      }
      // Rear
      /*
      try {
        UsbCamera camera = m_simpleCameraServer.startAutomaticCapture(CameraPort.REAR_CAMERA);
        camera.setResolution(640, 480);// 160,120
        camera.setFPS(30);
        System.out.println("Rear Camera valid? - " + camera.isValid());
        System.out.println("Rear Camera connected? - " + camera.isConnected());
      } catch (Exception ex) {
        DriverStation.reportError("Cannot start Rear CameraServer\n", false);
      }
      */
    }

    /* Chassis */
    m_chassis = new RobotChassis();
    m_shooter = new RobotShooter();
    m_driverJoystick = new Joystick(JoystickPort.PRIMARY_JOYSTICK);
    RobotDashboard.getInstance().setJoystick(m_driverJoystick);

    /* Dashboard */
    RobotDashboard.getInstance().setConfigValue(LABEL_AUTO_SIDE_INITIAL_DISTANCE, DEFAULT_AUTO_SIDE_INITIAL_DISTANCE);
    RobotDashboard.getInstance().setConfigValue(LABEL_GYRO_MULTIPLIER, DEFAULT_GYRO_MULTIPLIER);
    RobotDashboard.getInstance().setConfigValue(LABEL_AUTO_SIDE_LEFT_ANGLE, DEFAULT_AUTO_SIDE_LEFT_ANGLE);
    RobotDashboard.getInstance().setConfigValue(LABEL_AUTO_SIDE_RIGHT_ANGLE, DEFAULT_AUTO_SIDE_RIGHT_ANGLE);
    RobotDashboard.getInstance().setConfigValue(LABEL_AUTO_SIDE_FINAL_DISTANCE, DEFAULT_AUTO_SIDE_FINAL_DISTANCE);
    RobotDashboard.getInstance().setConfigValue(LABEL_AUTO_CENTER_DISTANCE, DEFAULT_AUTO_CENTER_DISTANCE);
  }

  /**
   * This autonomous (along with the m_chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * m_chooser code works with the Java SmartDashboard.
   * 
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the m_chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    AUTONOMOUS_COMPLETE = false;
  }

  /**
   * This function is called periodically during autonomous
   */
  @Override
  public void autonomousPeriodic() {
    if (AUTONOMOUS_COMPLETE) {
      m_chassis.arcadeDrive(0.0, 0.0);
      Timer.delay(0.05);
      return; 
    }

    System.out.println("Begin Auto");
    long abs_start = System.currentTimeMillis();
    AutoMode mode = RobotDashboard.getInstance().getAutoMode();
    
    do {
      // Set into Low Transmission.
      m_chassis.lowTransmission();
     
      double speed = AUTODRIVE_INITIAL_SPEED;

      
      // Scale the speed down to the [-1.0, 1.0] range.
      speed = Math.max(speed, -1.0);
      speed = Math.min(speed, 1.0);

      
      // Get the distance to be traveled from the dashboard.
      double distance = RobotDashboard.getInstance().getConfigValue(mode == AutoMode.CENTER ? LABEL_AUTO_CENTER_DISTANCE 
                                                                                            :LABEL_AUTO_SIDE_INITIAL_DISTANCE);
      // Turn this into seconds using some measured constants (& log the result)
      double timer = (distance * AUTODRIVE_10FEET_SECONDS) / AUTODRIVE_10FEET_INCHES;
      System.out.println("AutoDrive for " + timer + "s (" + distance + "\")");
      // Continue to Move forward for above calculated time
      long started = System.currentTimeMillis();
      
      m_chassis.resetGyro();
      //Timer.delay(0.25);
      double multiplier = RobotDashboard.getInstance().getConfigValue(LABEL_GYRO_MULTIPLIER);
      while ((System.currentTimeMillis() - started) / 1000.0 < timer) {
        /*
        if ((System.currentTimeMillis() - started) / 1000.0 > AUTODRIVE_GEAROUT_DELAY) {
          m_chassis.gearOut();
        }
        */
        m_chassis.arcadeDrive(-speed, -m_chassis.getGyroAngle() * multiplier);
        Timer.delay(0.01);
      }

      m_chassis.arcadeDrive(0.0, 0.0);
      
      if (mode == AutoMode.CENTER) { break; }
      
      // Turn
      m_chassis.resetGyro();
      Timer.delay(1.00);
      m_chassis.resetGyro();

      double turnSpeed = AUTODRIVE_TURN_POWER;
      double targetAngle = RobotDashboard.getInstance().getConfigValue(LABEL_AUTO_SIDE_LEFT_ANGLE);
      if (mode == AutoMode.RIGHT_SIDE) { 
        System.out.println("Inverting Speed & Target Angle for Right side");
        targetAngle = RobotDashboard.getInstance().getConfigValue(LABEL_AUTO_SIDE_RIGHT_ANGLE);
        turnSpeed = -turnSpeed;
        targetAngle = -targetAngle;
      }
      System.out.println("Starting turn!");

      double spinTimer = AUTODRIVE_TURN_SANITY_TIMER; // This timer is more of a safety
      long spinStarted = System.currentTimeMillis();  
      // Use a Deque of TimeEntries to comprise a history for the last tenth of a second to help normalize data.
      // We want to keep this minimal.
      History history = new History(100, false);
      history.appendToHistory(m_chassis.getGyroAngle());
      
      while ((mode == AutoMode.RIGHT_SIDE ? history.getHistoryAverage() > targetAngle
                                          : history.getHistoryAverage() < targetAngle)
             && (System.currentTimeMillis() - spinStarted) / 1000.0  < spinTimer) {
        //System.out.println("Gyro = "+m_chassis.getGyroAngle());
        history.appendToHistory(m_chassis.getGyroAngle());
        m_chassis.drive(-turnSpeed, turnSpeed);
        Timer.delay(0.01);
      }
      System.out.println("Done turning!");
      // When the timer has expired, stop.
      m_chassis.arcadeDrive(0.0, 0.0);
      
      // Step in
      m_chassis.resetGyro();
      // Attempt to fix the "hiccup"
      Timer.delay(1.00);
      m_chassis.resetGyro();
      double stepinDistance = RobotDashboard.getInstance().getConfigValue(LABEL_AUTO_SIDE_FINAL_DISTANCE);
      double stepinTimer = (stepinDistance * AUTODRIVE_10FEET_SECONDS) / AUTODRIVE_10FEET_INCHES;
      long stepinStarted = System.currentTimeMillis();
      while ((System.currentTimeMillis() - stepinStarted) / 1000.0 < stepinTimer) {
        m_chassis.arcadeDrive(-AUTODRIVE_FINAL_SPEED, //0.0);
             -m_chassis.getGyroAngle() * multiplier);
        Timer.delay(0.01);
      }
    } while (false);
    
    m_chassis.arcadeDrive(0.0, 0.0);
    System.out.println("Finished after " + (System.currentTimeMillis() - abs_start));


    
    System.out.println("End Auto");
    AUTONOMOUS_COMPLETE = true;
  }
  
  /**
   * This function is called at the beginning of TeleOp control
   */
  @Override
  public void teleopInit() {
    //m_driverJoystick = new Joystick(JoystickPort.PRIMARY_JOYSTICK);
  }

  
  /**
   * This function is called periodically during operator control
   */
  @Override
  public void teleopPeriodic() {

    while (isOperatorControl() && isEnabled()) {
      Timer.delay(0.005);
      m_chassis.periodic(m_driverJoystick);
      m_shooter.periodic(m_driverJoystick);
    }
  }

  /** ROBOT STATE */
  private RobotChassis m_chassis;
  private RobotShooter m_shooter;

  
  /** INPUT DEVICES */
  private Joystick     m_driverJoystick;
  private CameraServer m_simpleCameraServer;
  
  /* DO NOT TOUCH THESE VALUES! */
  private final double AUTODRIVE_10FEET_SECONDS    = 1.1; //seconds
  private final double AUTODRIVE_10FEET_INCHES     = 54;   //inches
  //private final double INITIAL_AUTO_RATIO = 2.044 / 88.8;
  private boolean      AUTONOMOUS_COMPLETE         = false;
}
