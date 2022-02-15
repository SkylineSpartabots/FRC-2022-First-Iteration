// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

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

    public static class ModulePorts{
        public int DRIVE_MOTOR;
        public int STEER_MOTOR;
        public int STEER_ENCODER;
        public double STEER_OFFSET;
        public ModulePorts(int driveMotor, int steerMotor, int steerEncoder, double steerOffset){
            this.DRIVE_MOTOR = driveMotor;
            this.STEER_MOTOR = steerMotor;
            this.STEER_ENCODER = steerEncoder;
            this.STEER_OFFSET = steerOffset;
        }
    }
    public static class DriveConstants{

        public static String nameForShuffleboardDebug = "none selected";
        
        public static int FRONT_LEFT_MODULE_DRIVE_MOTOR;
        public static int FRONT_LEFT_MODULE_STEER_MOTOR;
        public static int FRONT_LEFT_MODULE_STEER_ENCODER;
        public static double FRONT_LEFT_MODULE_STEER_OFFSET;

        public static int FRONT_RIGHT_MODULE_DRIVE_MOTOR;
        public static int FRONT_RIGHT_MODULE_STEER_MOTOR;
        public static int FRONT_RIGHT_MODULE_STEER_ENCODER;
        public static double FRONT_RIGHT_MODULE_STEER_OFFSET;

        public static int BACK_LEFT_MODULE_DRIVE_MOTOR;
        public static int BACK_LEFT_MODULE_STEER_MOTOR;
        public static int BACK_LEFT_MODULE_STEER_ENCODER;
        public static double BACK_LEFT_MODULE_STEER_OFFSET;

        public static int BACK_RIGHT_MODULE_DRIVE_MOTOR;
        public static int BACK_RIGHT_MODULE_STEER_MOTOR;
        public static int BACK_RIGHT_MODULE_STEER_ENCODER;
        public static double BACK_RIGHT_MODULE_STEER_OFFSET;

        public static double kTrackWidth;
        public static double kWheelBase;
        
        public static double ksVolts;
        public static double kvVoltSecondsPerMeter;
        public static double kaVoltSecondsSquaredPerMeter;

        public static double kVelocityGain;
        public static double kMaxAngularSpeedRadiansPerSecond;
        
        public DriveConstants(ModulePorts frontLeft, ModulePorts frontRight, ModulePorts backLeft, ModulePorts backRight){
            FRONT_LEFT_MODULE_DRIVE_MOTOR = frontLeft.DRIVE_MOTOR;
            FRONT_LEFT_MODULE_STEER_MOTOR = frontLeft.STEER_MOTOR;
            FRONT_LEFT_MODULE_STEER_ENCODER = frontLeft.STEER_ENCODER;
            FRONT_LEFT_MODULE_STEER_OFFSET = frontLeft.STEER_OFFSET;

            FRONT_RIGHT_MODULE_DRIVE_MOTOR = frontRight.DRIVE_MOTOR;
            FRONT_RIGHT_MODULE_STEER_MOTOR = frontRight.STEER_MOTOR;
            FRONT_RIGHT_MODULE_STEER_ENCODER = frontRight.STEER_ENCODER;
            FRONT_RIGHT_MODULE_STEER_OFFSET = frontRight.STEER_OFFSET;

            BACK_LEFT_MODULE_DRIVE_MOTOR = backLeft.DRIVE_MOTOR;
            BACK_LEFT_MODULE_STEER_MOTOR = backLeft.STEER_MOTOR;
            BACK_LEFT_MODULE_STEER_ENCODER = backLeft.STEER_ENCODER;
            BACK_LEFT_MODULE_STEER_OFFSET = backLeft.STEER_OFFSET;

            BACK_RIGHT_MODULE_DRIVE_MOTOR = backRight.DRIVE_MOTOR;
            BACK_RIGHT_MODULE_STEER_MOTOR = backRight.STEER_MOTOR;
            BACK_RIGHT_MODULE_STEER_ENCODER = backRight.STEER_ENCODER;
            BACK_RIGHT_MODULE_STEER_OFFSET = backRight.STEER_OFFSET;
        }
        
        public void setWheelBase(double wheelBase){
            kWheelBase = wheelBase;
        }

        public void setTrackWidth(double trackWidth){
            kTrackWidth = trackWidth;
        }

        public void setMaxAngularSpeed(){
            kMaxAngularSpeedRadiansPerSecond = kMaxSpeedMetersPerSecond / Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);
        }

        public void setCharacterizationConstants(double ks, double kv, double ka){
            ksVolts = ks;
            kvVoltSecondsPerMeter = kv;
            kaVoltSecondsSquaredPerMeter = ka;
        }


        public void setVelocityGain(double velocity){
            kVelocityGain = velocity;
        }
        public void setNameOfDriveConstant(String name){
            nameForShuffleboardDebug = name;
        }
        public static SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
        
        public static final double kMaxSpeedMetersPerSecond = 6380.0 / 60.0 *
            SdsModuleConfigurations.MK4_L2.getDriveReduction() *
            SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

        public static final double kMaxAccelerationMetersPerSecondSquared = 10; 
        public static final boolean kGyroReversed = false;

        public static final double DriveMaxAccelerationPerPeriodic =  20.0 / 200.0 ; //max acceleration, then, divide by 200 for 200 times per second
        public static final double RotationMaxAccelerationPerPeriodic =  100.0 / 200.0 ; //max acceleration, then, divide by 200 for 200 times per second
    }

    public static final class SmallDriveConstants {
        public static final ModulePorts FRONT_LEFT = new ModulePorts(0,1,8,-Math.toRadians(82.8781));
        public static final ModulePorts FRONT_RIGHT = new ModulePorts(2,3,9,-Math.toRadians(168.1320));
        public static final ModulePorts BACK_LEFT = new ModulePorts(4,5,10,-Math.toRadians(123.4863));
        public static final ModulePorts BACK_RIGHT = new ModulePorts(6,7,11,-Math.toRadians(157.2363));

        public static DriveConstants DRIVE_CONSTANTS = new DriveConstants(FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT);

        public SmallDriveConstants(){
            DRIVE_CONSTANTS.setTrackWidth(0.2921);
            DRIVE_CONSTANTS.setWheelBase(0.2921);
            DRIVE_CONSTANTS.setCharacterizationConstants(0.52717, 0.34344, 0.011638);

            DRIVE_CONSTANTS.setVelocityGain(4);
            DRIVE_CONSTANTS.setMaxAngularSpeed();
            DRIVE_CONSTANTS.setNameOfDriveConstant("Small Drive");
        }
    }

    public static final class ChronosDriveConstants{
        public static final ModulePorts FRONT_LEFT = new ModulePorts(2,1,9,-Math.toRadians(301.72576904296875));
        public static final ModulePorts FRONT_RIGHT = new ModulePorts(8,7,12,-Math.toRadians(265.9515380859375));
        public static final ModulePorts BACK_LEFT = new ModulePorts(4,3,10,-Math.toRadians(246.1761474609375));
        public static final ModulePorts BACK_RIGHT = new ModulePorts(6,5,11,-Math.toRadians(234.228515625));

        public static DriveConstants DRIVE_CONSTANTS = new DriveConstants(FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT);

        public ChronosDriveConstants(){
            DRIVE_CONSTANTS.setTrackWidth(0.4953);
            DRIVE_CONSTANTS.setWheelBase(0.4953);
            DRIVE_CONSTANTS.setCharacterizationConstants(0.50673, 0.34619, 0.018907);
            
            DRIVE_CONSTANTS.setVelocityGain(6);
            DRIVE_CONSTANTS.setMaxAngularSpeed();
            DRIVE_CONSTANTS.setNameOfDriveConstant("Chronos Drive");
        }
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
