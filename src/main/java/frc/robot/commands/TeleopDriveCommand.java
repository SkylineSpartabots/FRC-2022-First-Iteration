package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Controller;
import frc.robot.RobotContainer;
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
    SlewRateLimiter driveFilter = new SlewRateLimiter(0.5);
    SlewRateLimiter rotFilter = new SlewRateLimiter(0.5);

    public void driveWithJoystick() {
        // get joystick input for drive
        final var xSpeed = -modifyAxis(m_controller.getLeftY()) * DrivetrainSubsystem.MaxSpeedMetersPerSecond;
        final var ySpeed = -modifyAxis(m_controller.getLeftX()) * DrivetrainSubsystem.MaxSpeedMetersPerSecond;
        var rot = -modifyAxis(m_controller.getRightX()) * DrivetrainSubsystem.MaxAngularSpeedRadiansPerSecond;

        m_drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(driveFilter.calculate(xSpeed), driveFilter.calculate(ySpeed), 
                rotFilter.calculate(rot), m_drivetrainSubsystem.getGyroscopeRotation()));
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