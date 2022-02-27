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


import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;


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

  //SHUFFLEBOARD
  ShuffleboardTab t = Shuffleboard.getTab("Trajectory");
  NetworkTableEntry timeEntry = t.add("Elapsed Time", 0).getEntry();
  NetworkTableEntry accelEntry = t.add("Desired acceleration", 0).getEntry();
  NetworkTableEntry velEntry = t.add("Desired velocity", 0).getEntry();
  NetworkTableEntry curvEntry = t.add("Desired Curvature", 0).getEntry();
  NetworkTableEntry desTimeEntry = t.add("Desired Time", 0).getEntry();
  NetworkTableEntry endRotEntry = t.add("End Rotation",0).getEntry();
  NetworkTableEntry poseEntry = t.add("Desired Pose", 0).getEntry();
  NetworkTableEntry autoSpeedEntry = t.add("Auto Speed", 0).getEntry();
  NetworkTableEntry odoEntry = t.add("Odo ", 0).getEntry();
  

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

    t.add("Projected Time", m_trajectory.getTotalTimeSeconds());
    m_subsystem.getField().getObject("traj").setTrajectory(m_trajectory);//prints trajectory

    m_timer.reset();
    m_timer.start();

  }

  @Override
  public void execute() {

    var desiredSpeed  = m_trajectory.sample(m_timer.get());
    ChassisSpeeds targetChassisSpeeds = m_controller.calculate(
      new Pose2d(m_subsystem.getPose().getX(), m_subsystem.getPose().getY(), m_subsystem.getGyroscopeRotation()), desiredSpeed , m_endRotation);
    m_subsystem.drive(targetChassisSpeeds);

    //SHUFFLEBOARD
    timeEntry.setDouble(m_timer.get());
    accelEntry.setDouble(desiredSpeed.accelerationMetersPerSecondSq);
    velEntry.setDouble(desiredSpeed.velocityMetersPerSecond);
    curvEntry.setDouble(desiredSpeed.curvatureRadPerMeter);
    desTimeEntry.setDouble(desiredSpeed.timeSeconds);
    endRotEntry.setDouble(m_endRotation.getDegrees());
    poseEntry.setString(Math.round(desiredSpeed.poseMeters.getX()*100.0)/10.0 + ", " + 
      Math.round(desiredSpeed.poseMeters.getY()*100.0)/10.0 + ", " + 
      Math.round(desiredSpeed.poseMeters.getRotation().getDegrees()*100.0)/10.0);
    autoSpeedEntry.setString(Math.round(targetChassisSpeeds.vxMetersPerSecond*100.0)/10.0 + ", " + 
      Math.round(targetChassisSpeeds.vxMetersPerSecond*100.0)/10.0 + ", " + 
      Math.round(targetChassisSpeeds.omegaRadiansPerSecond*100.0)/10.0);
    odoEntry.setString(Math.round(m_subsystem.getPose().getX()*100.0)/10.0 + ", " + 
      Math.round(m_subsystem.getPose().getY()*100.0)/10.0 + ", " + 
      Math.round(m_subsystem.getPose().getRotation().getDegrees()*100.0)/10.0);
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
