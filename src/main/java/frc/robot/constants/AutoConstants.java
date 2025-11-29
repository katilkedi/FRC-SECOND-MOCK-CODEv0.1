package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.AngularAcceleration;
import static edu.wpi.first.units.Units.*;

public class AutoConstants {

    // Lineer hız ve ivme
    public static final LinearVelocity kMaxSpeedMetersPerSecond =
            MetersPerSecond.of(DrivetrainConstants.kPhysicalMaxSpeedMetersPerSecond / 4.0);
    public static final LinearAcceleration kMaxAccelerationMetersPerSecondSquared =
            MetersPerSecondPerSecond.of(3.0);

    // Açısal hız ve ivme
    public static final AngularVelocity kMaxAngularSpeedRadiansPerSecond =
            RadiansPerSecond.of(DrivetrainConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10.0);
    public static final AngularAcceleration kMaxAngularAccelerationRadiansPerSecondSquared =
            RadiansPerSecondPerSecond.of(Math.PI / 4.0);

    // PID sabitleri
    public static final double kPXController = 1.5;
    public static final double kPIController = 1.5;
    public static final double kPYController = 1.5;
    public static final double kPDController = 1.5;
    public static final double kPThetaController = 3.0;

    // Theta Controller Constraints
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                    kMaxAngularSpeedRadiansPerSecond.in(RadiansPerSecond),
                    kMaxAngularAccelerationRadiansPerSecondSquared.in(RadiansPerSecondPerSecond)
            );
}
