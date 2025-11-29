package frc.robot.constants;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import static edu.wpi.first.units.Units.*;

public final class ModuleConstants {

  // Wheel
  public static final Distance kWheelDiameter = Meters.of(0.10); // 0.10 m

  // Gear ratios
  public static final double kDriveMotorGearRatio = 1.0 / 6.75;
  public static final double kTurningMotorGearRatio = 1.0 / 12.8;

  // Encoder dönüşümleri
  public static final Distance kDriveEncoderRot2Meter =
            Meters.of(kDriveMotorGearRatio * Math.PI * kWheelDiameter.in(Meters));

  public static final Angle kTurningEncoderRot2Rad =
            Radians.of(kTurningMotorGearRatio * 2.0 * Math.PI);

  public static final LinearVelocity kDriveEncoderRPM2MeterPerSec =
            MetersPerSecond.of(kDriveEncoderRot2Meter.in(Meters) / 60.0);

  public static final AngularVelocity kTurningEncoderRPM2RadPerSec =
            RadiansPerSecond.of(kTurningEncoderRot2Rad.in(Radians) / 60.0);

  // PID
  public static final double kPTurning = 0.5;

}
