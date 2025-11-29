package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Distance;

public class DrivetrainConstants {

  public static final Distance kTrackWidth = Meters.of(0.536);   // 2 * 0.268
    public static final Distance kWheelBase = Meters.of(0.662);    // 2 * 0.331

    // Swerve kinematikleri
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase.div(2).in(Meters), kTrackWidth.div(-2).in(Meters)),
            new Translation2d(kWheelBase.div(2).in(Meters), kTrackWidth.div(2).in(Meters)),
            new Translation2d(kWheelBase.div(-2).in(Meters), kTrackWidth.div(-2).in(Meters)),
            new Translation2d(kWheelBase.div(-2).in(Meters), kTrackWidth.div(2).in(Meters))
    );

  public static final int kFrontLeftDriveMotorPort = 1;
  public static final int kBackLeftDriveMotorPort = 2;
  public static final int kFrontRightDriveMotorPort = 3;
  public static final int kBackRightDriveMotorPort = 4;

  public static final int kFrontLeftTurningMotorPort = 5;
  public static final int kBackLeftTurningMotorPort = 6;
  public static final int kFrontRightTurningMotorPort = 7;
  public static final int kBackRightTurningMotorPort = 8;

  public static final boolean kFrontLeftTurningEncoderReversed = true;
  public static final boolean kBackLeftTurningEncoderReversed = true;
  public static final boolean kFrontRightTurningEncoderReversed = true;
  public static final boolean kBackRightTurningEncoderReversed = true;

  public static final boolean kFrontLeftDriveEncoderReversed = true;
  public static final boolean kBackLeftDriveEncoderReversed = true;
  public static final boolean kFrontRightDriveEncoderReversed = false;
  public static final boolean kBackRightDriveEncoderReversed = false;

  public static final int kFrontLeftDriveAbsoluteEncoderPort = 1;
  public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
  public static final int kFrontRightDriveAbsoluteEncoderPort = 3;
  public static final int kBackRightDriveAbsoluteEncoderPort = 4;

  public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
  public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
  public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
  public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;
  
  public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 6.029;
  public static final double kBackLeftDriveAbsoluteEncoderOffsetRad  = 5.031;
  public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 4.467;
  public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 1.472;
  

  public static final double kPhysicalMaxSpeedMetersPerSecond = 5.0;
  public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 4 * Math.PI;

  public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4.0;
  public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4.0;
  public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3.0;
  public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3.0;
    }
