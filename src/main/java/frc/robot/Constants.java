package frc.robot;

public class Constants {
    public static final boolean tuningMode = true;

    // general PID constants - determined with SysId
    public static final double defaultkV = 0.11219;
    public static final double defaultkS = 0.003071;
    public static final double defaultkA = 0.016027;
    public static final double defaultkG = 8.5292E-05;

    // position constants
    public static final double defaultkPPos = 66.982;
    public static final double defaultkIPos = 0.0;
    public static final double defaultkDPos = 1.4;
    public static final int POSITION_GAIN_SLOT = 0;

    // velocity constants
    public static final double defaultkPVel = 0.1227;
    public static final double defaultkIVel = 0.0;
    public static final double defaultkDVel = 0.0;
    public static final int VELOCITY_GAIN_SLOT = 1;

    public static final double FALCON_500_FREE_SPEED = 6380.0;

    public enum ControlMode {
      Position("Position"), Velocity("Velocity");

      private final String name;
    
      ControlMode (String name) {
        this.name = name;
      }

      public String toString() {
        return name;
      }
    }
}
