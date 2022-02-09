// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.TurnConstants;
import frc.robot.commands.DriveCommandFactory;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.utils.Controller;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Button;

import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.ChassisSpeeds;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private final Controller m_controller = new Controller(new XboxController(0));

  private final ProfiledPIDController m_thetaController;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //For CAS PID Profiled alignment
    m_thetaController = new ProfiledPIDController(TurnConstants.kPThetaController, TurnConstants.kIThetaController, TurnConstants.kDThetaController, TurnConstants.kThetaControllerConstraints);
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

    m_drivetrainSubsystem = DrivetrainSubsystem.getInstance();

    m_drivetrainSubsystem.zeroGyroscope();
    // Set the scheduler to log Shuffleboard events for command initialize, interrupt, finish
    CommandScheduler.getInstance()
     .onCommandInitialize(
         command ->
             Shuffleboard.addEventMarker(
                 "Command initialized", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command ->
                Shuffleboard.addEventMarker(
                    "Command interrupted", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance()
        .onCommandFinish(
            command ->
                Shuffleboard.addEventMarker(
                    "Command finished", command.getName(), EventImportance.kNormal));

    // Configure the button bindings
    configureButtonBindings();

    DriveCommandFactory.swapAutonomousCommands();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    m_controller.getBackButton().whenPressed(m_drivetrainSubsystem::zeroGyroscope);
    m_controller.getAButton().whenPressed(this::resetOdometryFromPosition);
    m_controller.getXButton().whenPressed(this::softResetOdometryFromReference);
    m_controller.getButtonCombo(m_controller.getXButton(), m_controller.getYButton()).whenActive(this::hardResetOdometryFromReference);
  }

  public DrivetrainSubsystem getDriveTrainSubsystem()
  {
    return m_drivetrainSubsystem;
  }

  public void driveWithJoystick() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MaxSpeedMetersPerSecond;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MaxSpeedMetersPerSecond;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    var rot = -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MaxAngularSpeedRadiansPerSecond;

    m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_drivetrainSubsystem.getGyroscopeRotation()));
  }
  
  public void resetOdometryFromReference(double threshold){
    Translation2d current = m_drivetrainSubsystem.getPose().getTranslation();
    double minError = FieldConstants.kMinReferenceError;
    Translation2d newPos = null;
    for(Translation2d ref : FieldConstants.kReferenceTranslations){
      double errorX = Math.abs(ref.getX() - current.getX());
      double errorY = Math.abs(ref.getY() - current.getY());
      double error = Math.min(minError, Math.sqrt(Math.pow(errorX, 2) + Math.pow(errorY, 2)));
      if(error < minError){
        newPos = ref;
        minError = error;
      }
    }
    if(minError < FieldConstants.kMinReferenceError){
      m_drivetrainSubsystem.resetOdometry(new Pose2d(newPos, m_drivetrainSubsystem.getGyroscopeRotation()));
      SmartDashboard.putBoolean("Too Far From Reference", false);
    }
    else
      SmartDashboard.putBoolean("Too Far From Reference", true);
  }

  public void softResetOdometryFromReference(){
    resetOdometryFromReference(FieldConstants.kMinReferenceError);
  }
  public void hardResetOdometryFromReference(){
    resetOdometryFromReference(100d);
  }

  public void resetOdometryFromPosition(){
     m_drivetrainSubsystem.resetOdometry(new Pose2d());
  } 
  
  public static double applyDeadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = applyDeadband(value, 0.1);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
