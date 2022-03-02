// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

//holds robot-wide constants
public final class Constants {
    // timeout for CAN commands and error checking
    public static final int kTimeOutMs = 10;
    
    public static final double shootVelocityCondition = 1000;// CHANGE THIS VALUE
    public static final double shooterFire = 0.51;
    public static final double shooterRamp = 0.5;
    public static final double shooterIdle = 0.2;
    public static final double shooterOff = 0.0;

    public static final double indexerOff = 0.0;
    public static final double indexerUp = 0.4;
    public static final double indexerDown = -0.2;
    public static final double indexerFire = 0.6;

    public static final double intakeOn = 0.5;
    public static final double intakeOff = 0.0;
    public static final double intakeReverse = -0.5;

    public static final class DriveConstants {

        public static final double kTrackWidth = 0.4953;
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = 0.4953;
        // Distance between front and back wheels on robot


        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final boolean kGyroReversed = false;

        //Calculated via SysId
        public static final double ksVolts = 0.70541;
        public static final double kvVoltSecondsPerMeter = 0.33259;
        public static final double kaVoltSecondsSquaredPerMeter = 0.016433;

        //Tuned to taste for desired max velocity
        public static final double kVelocityGain = 6;

        // The maximum voltage that will be delivered to the drive motors.
        // This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
        public static final double kMaxVoltage = 12.0;

        // The maximum velocity of the robot in meters per second.
        // This is a measure of how fast the robot should be able to drive in a straight line.
        public static final double kMaxSpeedMetersPerSecond = 6380.0 / 60.0 *
               SdsModuleConfigurations.MK4_L2.getDriveReduction() *
               SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
     
       // need measure on robot
       public static final double kMaxAccelerationMetersPerSecondSquared = 10; 

       //The maximum angular velocity of the robot in radians per second.
       //This is a measure of how fast the robot can rotate in place.
       // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
       public static final double kMaxAngularSpeedRadiansPerSecond = kMaxSpeedMetersPerSecond /
              Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);
    }

    public static final class Ports{
        public static final int FRONT_LEFT_DRIVE = 2;
        public static final int FRONT_LEFT_STEER = 1;
        public static final int FRONT_LEFT_STEER_ENCODER = 9;
        public static final double FRONT_LEFT_OFFSET = -Math.toRadians(301.72576904296875);

        public static final int FRONT_RIGHT_DRIVE = 8;
        public static final int FRONT_RIGHT_STEER = 7;
        public static final int FRONT_RIGHT_STEER_ENCODER = 12;
        public static final double FRONT_RIGHT_OFFSET = -Math.toRadians(265.9515380859375);

        public static final int BACK_LEFT_DRIVE = 4;
        public static final int BACK_LEFT_STEER = 3;
        public static final int BACK_LEFT_STEER_ENCODER = 10;
        public static final double BACK_LEFT_OFFSET = -Math.toRadians(246.1761474609375);

        public static final int BACK_RIGHT_DRIVE = 6;
        public static final int BACK_RIGHT_STEER = 5;
        public static final int BACK_RIGHT_STEER_ENCODER = 11;
        public static final double BACK_RIGHT_OFFSET = -Math.toRadians(234.228515625);

        public static final int MASTER_SHOOTER_MOTOR = 21;
        public static final int FOLLOW_SHOOTER_MOTOR = 22;
        public static final int INTAKE_MOTOR = 31;
        public static final int INDEXER_MOTOR = 32;
    }
}
