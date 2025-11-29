package frc.robot.subsystems.drivetrain.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SimModule {
  private final Translation2d moduleLocation;
  private final double wheelRadiusMeters;

  private SwerveModuleState currentState = new SwerveModuleState();
  private double wheelLinearPositionMeters = 0.0; 
  private double azimuthRadians = 0.0; 

  public SimModule(Translation2d location, double wheelRadiusMeters) {
    this.moduleLocation = location;
    this.wheelRadiusMeters = wheelRadiusMeters;
  }

  public void setDesiredState(SwerveModuleState desired) {
    currentState = SwerveModuleState.optimize(desired, new Rotation2d(azimuthRadians));
  }

  public SwerveModuleState getState() {
    return currentState;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(wheelLinearPositionMeters, new Rotation2d(azimuthRadians));
  }

  public void simulationPeriodic(double dtSeconds) {
    double delta = currentState.speedMetersPerSecond * dtSeconds;
    wheelLinearPositionMeters += delta;
    azimuthRadians = currentState.angle.getRadians();
  }
}
