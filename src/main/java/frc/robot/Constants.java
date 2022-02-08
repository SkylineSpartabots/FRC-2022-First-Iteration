// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //timeout for CAN commands and error checking
    public static final int kTimeOutMs = 10;

    public static class DriveConstants {
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 0; // FIXME Set front left module drive motor ID
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 1; // FIXME Set front left module steer motor ID
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 8; // FIXME Set front left steer encoder ID
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(82.8781); // FIXME Measure and set front left steer offset

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 2; // FIXME Set front right drive motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 3; // FIXME Set front right steer motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 9; // FIXME Set front right steer encoder ID
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(168.1320); // FIXME Measure and set front right steer offset

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 4; // FIXME Set back left drive motor ID
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 5; // FIXME Set back left steer motor ID
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 10; // FIXME Set back left steer encoder ID
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(123.4863); // FIXME Measure and set back left steer offset

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6; // FIXME Set back right drive motor ID
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 7; // FIXME Set back right steer motor ID
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11; // FIXME Set back right steer encoder ID
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(157.2363); // FIXME Measure and set back right steer offset

        public static final double kTrackWidth = 0.2921;
        public static final double kWheelBase = 0.2921;

        public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

                public static final boolean kGyroReversed = false;

        public static final double ksVolts = 0.52717;
        public static final double kvVoltSecondsPerMeter = 0.34344;
        public static final double kaVoltSecondsSquaredPerMeter = 0.011638;
    
        public static final double kMaxSpeedMetersPerSecond = 6;
        
        public static final double DriveMaxAccelerationPerPeriodic =  20.0 / 200.0 ; //max acceleration, then, divide by 200 for 200 times per second
        public static final double RotationMaxAccelerationPerPeriodic =  100.0 / 200.0 ; //max acceleration, then, divide by 200 for 200 times per second
    }

    public static final class ChronosDriveConstants extends DriveConstants{
        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 2; // FIXME Set front left module drive motor ID
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 1; // FIXME Set front left module steer motor ID
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 9; // FIXME Set front left steer encoder ID
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(301.72576904296875); // FIXME Measure and set front left steer offset
    
        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8; // FIXME Set front right drive motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 7; // FIXME Set front right steer motor ID
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 12; // FIXME Set front right steer encoder ID
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(265.9515380859375); // FIXME Measure and set front right steer offset
    
        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 4; // FIXME Set back left drive motor ID
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 3; // FIXME Set back left steer motor ID
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 10; // FIXME Set back left steer encoder ID
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(246.1761474609375); // FIXME Measure and set back left steer offset
    
        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 6; // FIXME Set back right drive motor ID
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 5; // FIXME Set back right steer motor ID
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 11; // FIXME Set back right steer encoder ID86
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(234.228515625); // FIXME Measure and set back right steer offset
  
        public static final double kTrackWidth = 0.4953; // FIXME Measure and set trackwidth
        public static final double kWheelBase = 0.4953; // FIXME Measure and set wheelbase
        
        public static final double ksVolts = 0.50673;
        public static final double kvVoltSecondsPerMeter = 0.34619;
        public static final double kaVoltSecondsSquaredPerMeter = 0.018907;
    }
    
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class TurnConstants{
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 4;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 2;
        public static final double kPThetaController = 18.5;
        public static final double kIThetaController = 0;
        public static final double kDThetaController = 0.2;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class FieldConstants{
        //Meters, degrees
        public static final double kTargetHeight = 2.36474;
        public static final double kLensHeight = 2.36474;
        public static final double kLensAngle = 1;

        public static final Translation2d kTargetTranslation = new Translation2d(0,0);
        //TODO:
        //Determine starting field positions
        public static final Map<String, Pose2d> kStartingPositions = new HashMap(){{
            put("Blue Tarmac Inner", new Pose2d(0,0, new Rotation2d()));
            put("Blue Tarmac Outer", new Pose2d(0,0, new Rotation2d()));
            put("Red Tarmac Inner", new Pose2d(0,0, new Rotation2d()));
            put("Red Tarmac Outer", new Pose2d(0,0, new Rotation2d()));
        }};
        public static final List<Translation2d> kReferenceTranslations = List.of(
            new Translation2d(1, 0),
            new Translation2d(3, 3)
        );

        public static final double kMinReferenceError = 0.1;
    }
    public static final class ShooterConstants {
        public static final int MASTER_SHOOTER_MOTOR = 2;
        public static final int SLAVE_SHOOTER_MOTOR = 3;

        public static final double kFlywheelAngle = 0;
        public static final double kFlywheelHeight = 0;
        public static final double kFlywheetRobotOffset = 0; // Distance between front of shooter to front of robot
        public static final double kGravityAccelConstant = 9.81;

        public static final double kFalconVeloRpmFactor = 0;
        public static final double cargoMassKg = 35.274;
        public static final int kFlywheelMotor = 0;
        public static final double distanceToVelocity = 0.5379;
    }

    public static final class IndexerConstants {
        public static final int INDEXER_MOTOR = 9;
        public static final double indexerSpeedPercent = 0.3;
    }

    public static final class ClimbConstants {
        public static final double kClimbMaxHeight = 18800;
    }

    public static final class Ports{
        //TODO: 
        //GET PORTS FROM PHOENIX TUNER
        //ALSO I THINK WE'RE NOT SUPPOSED TO USE MASTER/SLAVE TERMINOLOGY SO REFACTOR THAT LATER
        public static final int CLIMB_ENCODER_A = 0;
        public static final int CLIMB_ENCODER_B = 0;
        public static final int CLIMB_HOOK_ID = 0;
        public static final int CLIMB_MASTER_WINCH_ID = 0;
        public static final int CLIMB_SLAVE_WINCH_ID = 0;
    }
}
