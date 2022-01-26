package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightSubsystem extends SubsystemBase {
    private static LimelightSubsystem instance = null;
    private NetworkTable nt;
    private boolean driverMode = false;

    public static LimelightSubsystem getInstance(){
        if(instance == null){
            instance = new LimelightSubsystem();
        }
        return instance;
    }

    public enum LimelightControl {
        LED_Default(0),
        LED_Off(1),
        LED_Blink(2),
        LED_On(3),
        Cam_Vision(0),
        Cam_Driver(1),
        Stream_Standard(0),
        Stream_PiPMain(1),
        Stream_PiPSecondary(2),
        Snapshot_Stop(0),
        Snapshot_Take(1);

        public int number(){
            return value;
        }
        int value;
        LimelightControl(int value){
            this.value = value;
        }
    }

    private LimelightSubsystem(){
        nt = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void init(){
        nt.getEntry("ledMode").setNumber(LimelightControl.LED_Default.number());
        nt.getEntry("camMode").setNumber(LimelightControl.Cam_Vision.number());
    }

    public void toggleDriveCam(){
        if(driverMode)
            nt.getEntry("camMode").setNumber(LimelightControl.Cam_Vision.number());
        else
            nt.getEntry("camMode").setNumber(LimelightControl.Cam_Vision.number());
        driverMode = !driverMode;
    }

    public boolean hasTarget(){
        return nt.getEntry("tv").getDouble(0.0) == 1.0;
    }

    public double getXOffset(){
        return nt.getEntry("tx").getDouble(0.0);
    }

    public double getYOffset(){
        return nt.getEntry("ty").getDouble(0.0);
    }
    
  @Override
  public void periodic() {
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", getXOffset());
    SmartDashboard.putNumber("LimelightY", getYOffset());
  }
  public Rotation2d getRotationOffsetFromPosition(Translation2d position){
      double targetDistance = getDistance();
      Translation2d targetPosition = FieldConstants.kTargetTranslation;
      double y = (position.getY() - targetPosition.getY()) / targetDistance;
      double x = (position.getX() - targetPosition.getX()) / targetDistance;
      double theta = Math.atan2(y, x);
      return new Rotation2d(theta);
  }
  public Translation2d getFieldPositionFromTarget(double radians){
       double d = getDistance();
       double x = d * Math.cos(radians - Math.PI);
       double y = d * Math.sin(radians - Math.PI);
       SmartDashboard.putNumber("DistanceLimelight", d);
       SmartDashboard.putNumber("XfromLimelight", x);
       SmartDashboard.putNumber("YfromLimelight", y);
       return new Translation2d(x, y);
  }
   public double getDistance() {
         double x = (FieldConstants.kTargetHeight - FieldConstants.kLensHeight) / 
             Math.tan(Math.toRadians(FieldConstants.kLensAngle + getYOffset()));
         x /= Math.cos(Math.toRadians(Math.abs(getXOffset())));
         return x;
   }
}