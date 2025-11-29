package frc.robot.subsystems.drivetrain.modules;

import edu.wpi.first.math.geometry.Rotation2d;

public class SimGyro {
  private double angleRadians = 0.0;
  private double angularVelocity = 0.0;

  public Rotation2d getRotation2d() {
    return new Rotation2d(angleRadians);
  }

  public double getAngleDegrees() {
    return Math.toDegrees(angleRadians);
  }
  
  public void simulate(double dt, double commandedOmega) {
    angularVelocity = commandedOmega;
    angleRadians += angularVelocity * dt;
  }
}
