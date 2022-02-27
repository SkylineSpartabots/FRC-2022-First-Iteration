package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Controller;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class TeleopDriveCommand extends CommandBase {
    private DrivetrainSubsystem m_drivetrainSubsystem;

    public TeleopDriveCommand(DrivetrainSubsystem m_drivetrainSubsystem) {
        this.m_drivetrainSubsystem = m_drivetrainSubsystem;
        addRequirements(m_drivetrainSubsystem);
        m_controller = RobotContainer.getController();
    }

    @Override
    public void execute() {
        driveWithJoystick();
    }

    private final Controller m_controller;

    //limit accel/deccel
    SlewRateLimiter driveXFilter = new SlewRateLimiter(7);
    SlewRateLimiter driveYFilter = new SlewRateLimiter(7);
    SlewRateLimiter rotFilter = new SlewRateLimiter(20);
    
    ShuffleboardTab t = Shuffleboard.getTab("Teleop Drive");
    NetworkTableEntry chassisEntry = t.add("Chassis Speed", 0).getEntry();
    NetworkTableEntry odoEntry = t.add("Odo Position", 0).getEntry();

    public void driveWithJoystick() {
        // get joystick input for drive
        var xSpeed = -modifyAxis(m_controller.getLeftY()) * DriveConstants.kMaxSpeedMetersPerSecond;
        var ySpeed = -modifyAxis(m_controller.getLeftX()) * DriveConstants.kMaxSpeedMetersPerSecond;
        var rot = -modifyAxis(m_controller.getRightX()) * DriveConstants.kMaxAngularSpeedRadiansPerSecond;

        m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(driveXFilter.calculate(xSpeed), driveYFilter.calculate(ySpeed), 
                rotFilter.calculate(rot), m_drivetrainSubsystem.getGyroscopeRotation()));
        ChassisSpeeds speeds = m_drivetrainSubsystem.getChassisSpeeds();

        //SHUFFLEBOARD
        odoEntry.setString(Math.round(m_drivetrainSubsystem.getPose().getX()*100.0)/10.0 + ", " + 
        Math.round(m_drivetrainSubsystem.getPose().getY()*100.0)/10.0 + ", " + 
        Math.round(m_drivetrainSubsystem.getPose().getRotation().getDegrees()*100.0)/10.0);
        chassisEntry.setString(Math.round(speeds.vxMetersPerSecond*100.0)/10.0 + ", " + 
        Math.round(speeds.vxMetersPerSecond*100.0)/10.0 + ", " + 
        Math.round(speeds.omegaRadiansPerSecond*100.0)/10.0);
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
        value = applyDeadband(value, 0.3);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }
}