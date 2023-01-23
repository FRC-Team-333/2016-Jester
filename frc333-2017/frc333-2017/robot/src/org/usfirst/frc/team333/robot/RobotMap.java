package org.usfirst.frc.team333.robot;

public class RobotMap {

  public static class VictorPort {
    public static final int CLIMBER_MOTOR = 0;
    public static final int DRIVE_LEFT_1  = 1;
    public static final int DRIVE_LEFT_2  = 2;
    public static final int DRIVE_LEFT_3  = 3;
    public static final int DRIVE_RIGHT_1 = 4;
    public static final int DRIVE_RIGHT_2 = 5;
    public static final int DRIVE_RIGHT_3 = 6;
    public static final int SHOOTER_MOTOR = -1;
    public static final int INTAKE_MOTOR  = 9;
  }

  public static class SolenoidPort {
    public static final int DRIVE_TRANS_LOW        = 0;
    public static final int DRIVE_TRANS_HIGH       = 1;
    public static final int GEAR_HOLDER_PISTON_IN  = 2;
    public static final int GEAR_HOLDER_PISTON_OUT = 3;
  }

  public static class GyroPort {
    public static final int MAIN_GYRO = 0;
  }

  public static class CompressorPort {
    public static final int MAIN_COMPRESSOR = 1;
  }

  public static class OperatorInputPort {
    public static final int DRIVER_STICK = 0;
  }

  public static class DigitalSensorPort {
    public static final int SHOOTER_RPM_SENSOR = 0;
  }

  public static class RelayPort {
    public static final int LIGHT_RELAY = 1;
  }
  
  public static class CameraPort {
    public static final int FRONT_CAMERA = 0;
    public static final int REAR_CAMERA  = 1;
  }
  
  public static class JoystickPort {
    public static final int PRIMARY_JOYSTICK = 0;
  }

  public static enum AutoMode {
    LEFT_SIDE,
    RIGHT_SIDE,
    CENTER
  }
  
  public static enum TransMode {
    MANUAL,
    AUTO
  }
}
