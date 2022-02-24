package frc.robot.commands;

import frc.robot.subsystems.*;

import static frc.robot.Constants.*;

import java.util.List;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class AutonomousCommandFactory {  

    //SendableChooser represents the command object and allows user to choose a specific program on the SmartDashboard
    public static SendableChooser<Command> m_chooser = new SendableChooser<>();

    //Called upon initialization of RobotContainer
    public static void swapAutonomousCommands(){
        m_chooser.setDefaultOption("Small Circle Auto", smallCircle());
        m_chooser.addOption("Big Circle Auto", bigCircle());
        m_chooser.addOption("Drive in line Auto", driveInLine());
    
        // Put the chooser on the dashboard
        SmartDashboard.putData(m_chooser);
  }
  
  public static Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  public static Command bigCircle() {
    var command =
      new TrajectoryDriveCommand(
          List.of(
            new Translation2d(1.5, 0),
            new Translation2d(1, 1.5),
            new Translation2d(0,1.5)
          ),
          new Pose2d(0, 0, new Rotation2d(0)),
          true);
    return command;
  }
  
  public static Command smallCircle() {
    var command =
      new TrajectoryDriveCommand(
          List.of(
            new Translation2d(0.5, 0),
            new Translation2d(0.5, 0.5),
            new Translation2d(0,0.5)
          ),
          new Pose2d(0, 0, new Rotation2d(0)),
          true);
    return command;
  }
  
  public static Command driveInLine() {
    var command =
      new TrajectoryDriveCommand(
          List.of(
            new Translation2d(2, 0)
          ),
          new Pose2d(0, 0, new Rotation2d(0)),
          true);
    return command;
  }


}