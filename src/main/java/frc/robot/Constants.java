package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {

    public static final class DriveConstants {

        public static final double kMaxSpeedMetersPerSecond = 4.8;
        public static final double kMaxAngularSpeed = 2 * Math.PI;

        public static final double kDirectionSlewRate = 1.2;
        public static final double kMagnitudeSlewRate = 1.8;
        public static final double kRotationalSlewRate = 2.0;

        public static final double kTrackWidth = Units.inchesToMeters(25.5);

        public static final double kWheelBase = Units.inchesToMeters(25.5);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kRearLeftChassisAngularOffset = Math.PI;
        public static final double kRearRightChassisAngularOffset = Math.PI / 2;

        public static final int kFrontLeftDrivingCanId = 10;
        public static final int kFrontRightDrivingCanId = 20;
        public static final int kRearLeftDrivingCanId = 30;
        public static final int kRearRightDrivingCanId = 40;

        public static final int kFrontLeftTurningCanId = 11;
        public static final int kFrontRightTurningCanId = 21;
        public static final int kRearLeftTurningCanId = 31;
        public static final int kRearRightTurningCanId = 41;

        public static final boolean kGyroReversed = false;
        public static double kBackLeftChassisAngularOffset;
        public static double kBackRightChassisAngularOffset;

    }

    public static final class ModuleContants {

        public static final int kDrivingMotorPinionTeeth = 14;

        public static final boolean kTurningEncoderInverted = true;

        public static final double kDrivingMotorFreeSpeedRps = NeoMotorContants.kFreeSpeenRpm / 60;
        public static final double kWheelDiameterMerters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMerters * Math.PI;

        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMerters * Math.PI) / kDrivingMotorReduction;
        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMerters * Math.PI) / kDrivingMotorReduction) / 60.0;

        public static final double kTurningEncoderPostionFactor = (2 * Math.PI);
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0;

        public static final double kTurningEncoderPostionPIDMinInput = 0;
        public static final double kTurningEncoderPostionPIDMaxInput = kTurningEncoderPostionFactor;

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 0;
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 40;
        public static final int kTurningMotorCurrentLimit = 20;

    }

    public static final class OIContants {

        public static final int kDriverControllerport = 0;
        public static final double kDrivedeadband = 0.05;
        public static final int kOPController = 1;
    
    }

    public static final class AutoContants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecond = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAccelerationMetersPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kThetaController = 1;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, 
            kMaxAccelerationMetersPerSecondSquared);
    
        
    }

    public static final class NeoMotorContants {

        public static final double kFreeSpeenRpm = 5676;
        
    }
    
}
