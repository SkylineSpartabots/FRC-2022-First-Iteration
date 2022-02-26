package frc.robot.factories;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.TrajectoryDriveCommand;
import frc.robot.commands.SetSubsystemCommand.*;
import frc.robot.subsystems.DrivetrainSubsystem;
import static frc.robot.Constants.*;

public class AutonomousCommandFactory {

    // SendableChooser represents the command object and allows user to choose a
    // specific program on the SmartDashboard
    public static SendableChooser<Command> m_chooser = new SendableChooser<>();

    // Called upon initialization of RobotContainer
    public static void swapAutonomousCommands() {
        m_chooser.setDefaultOption("Blue Four Ball Auto Bottom Left", blueFourBallAuto());
        //m_chooser.addOption("Big Circle Auto", bigCircle());

        // Put the chooser on the dashboard
        SmartDashboard.putData(m_chooser);
    }

    public static Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

    public static Command blueFourBallAuto(){
        DrivetrainSubsystem m_drivetrainSubsystem = DrivetrainSubsystem.getInstance();

        final double startX = 7.70;
        final double startY = 2.80;
        final double startRot = -110;
        final double pos1X = 8.9;
        final double pos1Y = 4.9;
        final double pos1Rot = 336;
        final double pos2X = 1.56;
        final double pos2Y = 1.55;
        final double pos2Rot = -135;
        final double pos3X = 7.63;
        final double pos3Y = 0.78;
        final double pos3Rot = -90;

        //SET STARTING POSITION
        m_drivetrainSubsystem.resetOdometryFromPosition(startX,startY,startRot);

        //startX, startY, startRot, endX, endY, endRot
       // Command turnOnIntake = new SetIntakeCommand(intakeOn);
       // Command rampUpShooter = new SetShooterCommand(shooterRamp);
        Command driveToFirstBall = new TrajectoryDriveCommand(pos1X, pos1Y, pos1Rot);
       /* Command driveBackToShoot = new TrajectoryDriveCommand(pos1X, pos1Y, pos1Rot, startX, startY, startRot);
        Command fireIndexer = new SetIndexerCommand(indexerFire);
        Command waitForShooterToFinish = new WaitCommand(2);
        Command turnOffIndexer = new SetIndexerCommand(indexerOff);
        Command driveToSecondBall = new TrajectoryDriveCommand(startX, startY, startRot, pos2X, pos2Y, pos2Rot);
        Command driveBackToShootSecondTime = new TrajectoryDriveCommand(pos2X, pos2Y, pos2Rot, startX, startY, startRot);
        Command fireIndexer2 = new SetIndexerCommand(indexerFire);
        Command waitForShooterToFinish2 = new WaitCommand(2);
        Command turnOffIndexer2 = new SetIndexerCommand(indexerOff);
        Command turnOffShooter = new SetShooterCommand(shooterOff);
        Command driveToThirdBall = new TrajectoryDriveCommand(startX, pos2Y, startY, pos3X, pos3Y, pos3Rot);
        Command turnIntakeOff = new SetIndexerCommand(intakeOff);
*/
        return new SequentialCommandGroup(
         //   turnOnIntake,
         //   rampUpShooter,
            driveToFirstBall
          /*  driveBackToShoot,
            fireIndexer,
            waitForShooterToFinish,
            turnOffIndexer,
            driveToSecondBall,
            driveBackToShootSecondTime,
            fireIndexer2,
            waitForShooterToFinish2,
            turnOffIndexer2,
            turnOffShooter,
            driveToThirdBall,
            turnIntakeOff*/
        );
    }

}