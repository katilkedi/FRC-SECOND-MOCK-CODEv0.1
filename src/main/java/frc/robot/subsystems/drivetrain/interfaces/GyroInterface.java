package frc.robot.subsystems.drivetrain.interfaces;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroInterface {
  public Rotation2d getAngle();

  public void setAngle(Rotation2d angle);
}