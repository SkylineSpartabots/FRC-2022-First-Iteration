package frc.robot.factories;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.TrajectoryDriveCommand;
import frc.robot.commands.SetSubsystemCommand.*;
import frc.robot.subsystems.DrivetrainSubsystem;
import static frc.robot.Constants.*;

import java.util.List;

public class AutonomousCommandFactory {

    // SendableChooser represents the command object and allows user to choose a
    // specific program on the SmartDashboard
    public static SendableChooser<Command> m_chooser = new SendableChooser<>();

    // Called upon initialization of RobotContainer
    public static void swapAutonomousCommands() {
        m_chooser.setDefaultOption("Blue Four Ball Auto Bottom Left", blueFourBallAuto());

        // Put the chooser on the dashboard
        SmartDashboard.putData(m_chooser);
    }

    public static Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }


    public static Command blueFourBallAuto(){
        DrivetrainSubsystem m_drivetrainSubsystem = DrivetrainSubsystem.getInstance();

        final double startX = 7.82;
        final double startY = 2.97;
        final double startRot = -110;
        final double pos1X = 5.36;
        final double pos1Y = 1.98;
        final double pos1Rot = -160;
        final double pos2X = 1.45;
        final double pos2Y = 1.37;
        final double pos2Rot = -137;
        final double pos3X = 7.63;
        final double pos3Y = 0.65;
        final double pos3Rot = -90;
        
        //SET STARTING POSITION
        Command resetOdo = new InstantCommand(() ->  m_drivetrainSubsystem.resetOdometryFromPosition(startX, startY, startRot), m_drivetrainSubsystem);

        //startX, startY, startRot, endX, endY, endRot
        Command turnOnIntake = new SetIntakeCommand(intakeOn);
        Command rampUpShooter = new SetShooterCommand(shooterRamp);
        Command driveToFirstBall = new TrajectoryDriveCommand(pos1X, pos1Y, pos1Rot, List.of(), false);
        Command driveBackToShoot = new TrajectoryDriveCommand(startX, startY, startRot,List.of(), true);
        Command fireIndexer = new SetIndexerCommand(indexerFire);
        Command waitForShooterToFinish = new WaitCommand(2);
        Command turnOffIndexer = new SetIndexerCommand(indexerOff);
        Command driveToSecondBall = new TrajectoryDriveCommand(pos2X, pos2Y, pos2Rot,List.of(new Translation2d(4.25, 1.52)), false);
        Command driveBackToShootSecondTime = new TrajectoryDriveCommand(startX, startY, startRot,List.of(new Translation2d(4.25, 1.52)),true);
        Command fireIndexer2 = new SetIndexerCommand(indexerFire);
        Command waitForShooterToFinish2 = new WaitCommand(2);
        Command turnOffIndexer2 = new SetIndexerCommand(indexerOff);
        Command turnOffShooter = new SetShooterCommand(shooterOff);
        Command driveToThirdBall = new TrajectoryDriveCommand(pos3X, pos3Y, pos3Rot,List.of(), false);
        Command turnIntakeOff = new SetIntakeCommand(intakeOff);

        return new SequentialCommandGroup(
            resetOdo,
            turnOnIntake,
            rampUpShooter,
            driveToFirstBall,
            driveBackToShoot,
            fireIndexer,
            waitForShooterToFinish,
            turnOffIndexer,
            driveToSecondBall,
            driveBackToShootSecondTime,
            fireIndexer2,
            waitForShooterToFinish2,
            turnOffIndexer2,
            turnOffShooter,
            //driveToThirdBall,
            turnIntakeOff
        );
    }

}