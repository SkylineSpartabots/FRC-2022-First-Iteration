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

import java.util.ArrayList;
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

    public static Pose2d getPose(double x, double y, double rot){
       return new Pose2d(x, y, new Rotation2d(Math.toRadians(rot)));
    }
    public static Command blueFourBallAuto(){
        DrivetrainSubsystem m_drivetrainSubsystem = DrivetrainSubsystem.getInstance();

        Pose2d startPose = getPose(7.7, 2.8, -110);
        Pose2d ball1 = getPose(5.41, 1.76, 145);
        Pose2d ball2 = getPose(1.56, 1.55, -135);
        Pose2d ball3 = getPose(7.63, 0.78, -90);

        //SET STARTING POSITION
        Command resetOdo = new InstantCommand(() ->  m_drivetrainSubsystem.resetOdometryFromPosition(startPose), m_drivetrainSubsystem);

        //startX, startY, startRot, endX, endY, endRot
        Command turnOnIntake = new SetIntakeCommand(intakeOn);
        Command rampUpShooter = new SetShooterCommand(shooterRamp);
        Command driveToFirstBall = new TrajectoryDriveCommand(ball1, List.of(new Translation2d(6.29, 1.37)), false);
        Command driveBackToShoot = new TrajectoryDriveCommand(startPose,List.of(new Translation2d(6.29, 1.37)), true);
        Command fireIndexer = new SetIndexerCommand(indexerFire);
        Command waitForShooterToFinish = new WaitCommand(2);
        Command turnOffIndexer = new SetIndexerCommand(indexerOff);
        Command driveToSecondBall = new TrajectoryDriveCommand(ball2, List.of(new Translation2d(4.25, 1.52)), false);
        Command driveBackToShootSecondTime = new TrajectoryDriveCommand(startPose,List.of(new Translation2d(4.25, 1.52)),true);
        Command fireIndexer2 = new SetIndexerCommand(indexerFire);
        Command waitForShooterToFinish2 = new WaitCommand(2);
        Command turnOffIndexer2 = new SetIndexerCommand(indexerOff);
        Command turnOffShooter = new SetShooterCommand(shooterOff);
        Command driveToThirdBall = new TrajectoryDriveCommand(ball3, List.of(), false);
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