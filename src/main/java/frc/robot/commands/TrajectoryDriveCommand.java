// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;


public class TrajectoryDriveCommand extends CommandBase {
  private final DrivetrainSubsystem m_subsystem;

  private Trajectory m_trajectory;
  private Rotation2d m_endRotation;
  private HolonomicDriveController m_controller;

  //SET MAX SPEED AND MAX ACCELERATION
  private final double maxSpeed = 1;
  private final double maxAcceleration = 1;


  public TrajectoryDriveCommand(double startX, double startY, double startRotation, double endX, double endY, double endRotation){
      m_subsystem = DrivetrainSubsystem.getInstance();
      addRequirements(m_subsystem); //add requirements

      TrajectoryConfig config = new TrajectoryConfig(maxSpeed, maxAcceleration).setKinematics(DriveConstants.kDriveKinematics)
        .setStartVelocity(0).setEndVelocity(0);
      m_trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(startX,startY, new Rotation2d(Math.toRadians(startRotation))), List.of(),
        new Pose2d(endX, endY, new Rotation2d(Math.toRadians(endRotation))), config);
  }

  public TrajectoryDriveCommand(Trajectory p_trajectory) {
    m_subsystem = DrivetrainSubsystem.getInstance();
    addRequirements(m_subsystem); //add requirements

    m_trajectory = p_trajectory;
  }


  private final Timer m_timer = new Timer();

  @Override
  public void initialize() {
    PIDController xController = new PIDController(1, 0, 0);
    PIDController yController = new PIDController(1, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    m_controller = new HolonomicDriveController(xController, yController, thetaController);
    m_controller.setEnabled(true);

    Pose2d endPose = m_trajectory.getStates().get(m_trajectory.getStates().size() - 1).poseMeters;
    m_endRotation = endPose.getRotation();
    m_timer.reset();
    m_timer.start();
  }

  @Override
  public void execute() {
    ChassisSpeeds targetChassisSpeeds = m_controller.calculate(m_subsystem.getPose(), m_trajectory.sample(m_timer.get()), m_endRotation);
    m_subsystem.drive(targetChassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_subsystem.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }
}
