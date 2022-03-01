// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.lib.util.Controller;
import frc.robot.commands.SetSubsystemCommand.SetIndexerCommand;
import frc.robot.commands.SetSubsystemCommand.SetIntakeCommand;
import frc.robot.commands.SetSubsystemCommand.SetShooterCommand;
import frc.robot.factories.AutonomousCommandFactory;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private DrivetrainSubsystem m_drivetrainSubsystem;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_drivetrainSubsystem = DrivetrainSubsystem.getInstance();

    // Set the scheduler to log Shuffleboard events for command initialize,
    // interrupt, finish
    CommandScheduler.getInstance().onCommandInitialize(command -> Shuffleboard.addEventMarker(
        "Command initialized", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance().onCommandInterrupt(command -> Shuffleboard.addEventMarker(
        "Command interrupted", command.getName(), EventImportance.kNormal));
    CommandScheduler.getInstance().onCommandFinish(command -> Shuffleboard.addEventMarker(
        "Command finished", command.getName(), EventImportance.kNormal));

    // Configure the button bindings
    configureButtonBindings();

    // initialize Shuffleboard swapping of autonomous commands
    AutonomousCommandFactory.swapAutonomousCommands();
  }

  private static final Controller m_controller = new Controller(new XboxController(0));

  public static Controller getController(){
    return m_controller;
  }

  // configures button bindings to controller
  private void configureButtonBindings() {
    final double triggerDeadzone = 0.8;

    // back button
    m_controller.getBackButton().whenPressed(m_drivetrainSubsystem::resetOdometry);// resets odometry and heading
    m_controller.getXButton().whenPressed(m_drivetrainSubsystem::resetFromStart);// resets odometry and heading

     //left triggers and bumpers
     Trigger leftTriggerAxis = new Trigger(() -> { return m_controller.getLeftTriggerAxis() > triggerDeadzone;});//left trigger deadzone 0.8
     leftTriggerAxis.whenActive(new SetShooterCommand(shooterRamp));//on trigger hold
     leftTriggerAxis.whenInactive(new SetShooterCommand(shooterIdle));//on trigger release
     m_controller.getLeftBumper().whenPressed(new SetShooterCommand(shooterOff));//left bumper stops shooter
     
     //right triggers and bumpers
     Trigger rightTriggerAxis = new Trigger(() -> { return m_controller.getRightTriggerAxis() > triggerDeadzone;});//right trigger deadzone 0.8
     rightTriggerAxis.whenActive(//TO DO: FIGURE OUT CANCELLING COMMAND
        new SequentialCommandGroup( //on trigger hold, waits for
          new SetShooterCommand(shooterFire), //ramps up shooter to shooting speeds
          new WaitUntilCommand(() -> {return ShooterSubsystem.getInstance().shooterAtVelocityRPS(shootVelocityCondition);}), //waits for correct velocity
          new SetIndexerCommand(indexerFire), new SetIntakeCommand(intakeOn))); //fires indexer
     rightTriggerAxis.whenInactive(new ParallelCommandGroup(
        new SetShooterCommand(shooterIdle),
        new SetIndexerCommand(indexerOff),
        new SetIntakeCommand(intakeOff)));//on trigger release
     m_controller.getRightBumper().whenActive(new SetIndexerCommand(indexerUp));//right bumper hold
     m_controller.getRightBumper().whenInactive(new SetIndexerCommand(indexerOff));//right bumper release
     
    // buttons
    m_controller.getAButton().whenPressed(new SetIntakeCommand(intakeOn));
    m_controller.getYButton().whenPressed(new SetIntakeCommand(intakeReverse));
    m_controller.getYButton().whenReleased(new SetIntakeCommand(intakeOn));
    m_controller.getBButton().whenPressed(new SetIntakeCommand(intakeOff));
    
    //DPAD
    Trigger dpadUp = new Trigger(() -> {return m_controller.getDpadUp();});//hold dpad up for indexer up
    dpadUp.whenActive(new SetIndexerCommand(indexerUp)).whenInactive(new SetIndexerCommand(indexerOff));
    Trigger dpadDown = new Trigger(() -> {return m_controller.getDpadDown();});//hold dpad down for indexer down
    dpadDown.whenActive(new SetIndexerCommand(indexerDown)).whenInactive(new SetIndexerCommand(indexerOff));
    
  }

  public void onRobotDisabled() {
    //called when robot is disabled. Set all subsytems to 0
    IntakeSubsystem.getInstance().setIntakePercentPower(0.0);
    IndexerSubsystem.getInstance().setIndexerPercentPower(0.0);
    ShooterSubsystem.getInstance().setShooterPercentPower(0.0);
  }
}
