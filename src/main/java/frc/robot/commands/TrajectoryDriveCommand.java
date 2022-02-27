// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.List;

import javax.swing.plaf.TreeUI;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class TrajectoryDriveCommand extends CommandBase {
  private final DrivetrainSubsystem m_subsystem;

  private Trajectory m_trajectory;
  private Rotation2d m_endRotation;
  private HolonomicDriveController m_controller;

  //SET MAX SPEED AND MAX ACCELERATION
  private final double maxSpeed = 2;
  private final double maxAcceleration = 2;
  

  double endX, endY, endRotation;
  List<Translation2d> interiorPoints;
  boolean reverse;
  public TrajectoryDriveCommand(double endX, double endY, double endRotation, List<Translation2d> interiorPoints, boolean reverse){
      this();
      this.endX = endX;
      this.endY = endY;
      this.endRotation = endRotation;
      this.interiorPoints = interiorPoints;
      this.reverse = reverse;
  }

  public TrajectoryDriveCommand(Pose2d endPose, List<Translation2d> interiorPoints, boolean reverse){
    this();
    this.endX = endPose.getX();
    this.endY = endPose.getY();
    this.endRotation = endPose.getRotation().getDegrees();
    this.interiorPoints = interiorPoints;
    this.reverse = reverse;
}
/*
  public TrajectoryDriveCommand(Trajectory p_trajectory) {
      this();
      m_trajectory = p_trajectory;
      Pose2d endPose = m_trajectory.getStates().get(m_trajectory.getStates().size() - 1).poseMeters;
      m_endRotation = endPose.getRotation();
  }*/

  private TrajectoryDriveCommand(){
    m_subsystem = DrivetrainSubsystem.getInstance();
    addRequirements(m_subsystem); //add requirements
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

    TrajectoryConfig config = new TrajectoryConfig(maxSpeed, maxAcceleration).setKinematics(DriveConstants.kDriveKinematics).setReversed(reverse);
    
    m_trajectory = TrajectoryGenerator.generateTrajectory(m_subsystem.getPose(), interiorPoints,
      new Pose2d(endX, endY, new Rotation2d(Math.toRadians(endRotation))), config);
    m_endRotation = new Rotation2d(Math.toRadians(endRotation));

    SmartDashboard.putNumber("Projected Time", m_trajectory.getTotalTimeSeconds());
    m_timer.reset();
    m_timer.start();

    m_subsystem.m_field.getObject("traj").setTrajectory(m_trajectory);

  }

  @Override
  public void execute() {

    var desiredSpeed  = m_trajectory.sample(m_timer.get());
    ChassisSpeeds targetChassisSpeeds = m_controller.calculate(
      new Pose2d(m_subsystem.getPose().getX(), m_subsystem.getPose().getY(), m_subsystem.getGyroscopeRotation()), desiredSpeed , m_endRotation);
    m_subsystem.drive(targetChassisSpeeds);

    SmartDashboard.putNumber("Elapsed Time", m_timer.get());
    SmartDashboard.putNumber("Desired acceleration", desiredSpeed.accelerationMetersPerSecondSq);
    SmartDashboard.putNumber("Desired velocity", desiredSpeed.velocityMetersPerSecond);
    SmartDashboard.putNumber("DesiredX", desiredSpeed.poseMeters.getX());
    SmartDashboard.putNumber("DesiredY", desiredSpeed.poseMeters.getY());
    SmartDashboard.putNumber("DesiredRot", desiredSpeed.poseMeters.getRotation().getDegrees());
    SmartDashboard.putNumber("Desired Time", desiredSpeed.timeSeconds);
    SmartDashboard.putNumber("End Rotation", m_endRotation.getDegrees());
    SmartDashboard.putNumber("Auto X Speed", targetChassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Auto Y Speed", targetChassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Auto Rot Speed", targetChassisSpeeds.omegaRadiansPerSecond);
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
