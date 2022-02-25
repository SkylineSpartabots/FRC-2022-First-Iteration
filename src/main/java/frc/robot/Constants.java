// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    public static final double intakeOn = 0.8;
    public static final double intakeOff = 0.0;
    public static final double intakeReverse = -0.5;

    public static final class DriveConstants {

        public static final double kTrackWidth = 0.2921;
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = 0.2921;
        // Distance between front and back wheels on robot

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final boolean kGyroReversed = false;

    }

}
