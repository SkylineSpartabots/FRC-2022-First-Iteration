
package frc.robot.subsystems;

import static frc.robot.Constants.*;


import com.kauailabs.navx.frc.AHRS;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.TeleopDriveCommand;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DrivetrainSubsystem extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MaxSpeedMetersPerSecond = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK4_L2.getDriveReduction() *
          SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

  // need measure on robot
  public static final double MaxAccelerationMetersPerSecondSquared = 10; 

  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MaxAngularSpeedRadiansPerSecond = MaxSpeedMetersPerSecond /
          Math.hypot(DriveConstants.kTrackWidth / 2.0, DriveConstants.kWheelBase / 2.0);

  private final SwerveDriveOdometry m_odometry;
  private final SimpleMotorFeedforward m_feedforward;

  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  private static DrivetrainSubsystem m_instance = null;
  public static DrivetrainSubsystem getInstance(){
          if (m_instance == null) {
                m_instance = new DrivetrainSubsystem();
          }
          return m_instance;
  }

  public DrivetrainSubsystem() {
    setDefaultCommand(new TeleopDriveCommand(this));
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0),
            // This can either be STANDARD or FAST depending on your gear configuration
            Mk4SwerveModuleHelper.GearRatio.L2,
            // Port ID of drive motor, steer motor, steer encoder offset
            2, 1, 9, -Math.toRadians(301.72576904296875));         
            // Steer Encoder Offset: This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
    

    // We will do the same for the other modules
    m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0),
            Mk4SwerveModuleHelper.GearRatio.L2, 8, 7, 12, -Math.toRadians(265.9515380859375));

    m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(4, 0),
            Mk4SwerveModuleHelper.GearRatio.L2, 4, 3, 10, -Math.toRadians(246.1761474609375));

    m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList).withSize(2, 4).withPosition(6, 0),
            Mk4SwerveModuleHelper.GearRatio.L2, 6, 5, 11, -Math.toRadians(234.228515625));

    m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getGyroscopeRotation());
    m_feedforward = new SimpleMotorFeedforward(0.50673, 0.34619, 0.018907);
    
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    m_navx.zeroYaw();
  }

  private double rotationOffset = 0;
  public Rotation2d getGyroscopeRotation() {
   if (m_navx.isMagnetometerCalibrated()) {
     // We will only get valid fused headings if the magnetometer is calibrated
     return Rotation2d.fromDegrees(m_navx.getFusedHeading() + rotationOffset);
   }
   // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
   return Rotation2d.fromDegrees((-m_navx.getYaw())+rotationOffset);
  }
  
  public Pose2d getPose(){
        return m_odometry.getPoseMeters();
  }

  //resets to 0,0
  public void resetOdometry() {
    m_navx.reset();
    rotationOffset = 0;
    m_odometry.resetPosition(new Pose2d(), new Rotation2d(rotationOffset));
  }
  
  //resets from offset
  public void resetOdometryFromPosition(double x, double y, double rot) {
    m_navx.reset();
    rotationOffset = rot;
    m_odometry.resetPosition(new Pose2d(x,y,new Rotation2d(rot)), new Rotation2d(rotationOffset));
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }


  @Override
  public void periodic() {    
    var pose = m_odometry.getPoseMeters();

    SmartDashboard.putNumber("X Position", pose.getTranslation().getX());
    SmartDashboard.putNumber("Y Position", pose.getTranslation().getY());
    SmartDashboard.putNumber("Rotation", getGyroscopeRotation().getDegrees());
  }

  public void applyDrive() {
    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MaxSpeedMetersPerSecond);

    m_frontLeftModule.set(getVoltageByVelocity(states[0].speedMetersPerSecond), states[0].angle.getRadians());
    m_frontRightModule.set(getVoltageByVelocity(states[1].speedMetersPerSecond), states[1].angle.getRadians());
    m_backLeftModule.set(getVoltageByVelocity(states[2].speedMetersPerSecond), states[2].angle.getRadians());
    m_backRightModule.set(getVoltageByVelocity(states[3].speedMetersPerSecond), states[3].angle.getRadians());

    m_odometry.update(getGyroscopeRotation(), 
        new SwerveModuleState(m_frontLeftModule.getDriveVelocity(), new Rotation2d(m_frontLeftModule.getSteerAngle())),
        new SwerveModuleState(m_frontRightModule.getDriveVelocity(), new Rotation2d(m_frontRightModule.getSteerAngle())),
        new SwerveModuleState(m_backLeftModule.getDriveVelocity(), new Rotation2d(m_backLeftModule.getSteerAngle())),
        new SwerveModuleState(m_backRightModule.getDriveVelocity(), new Rotation2d(m_backRightModule.getSteerAngle())));
  }
  
  public double getVoltageByVelocity(double targetVelocity){
    return m_feedforward.calculate(targetVelocity * 6);
  }
}
