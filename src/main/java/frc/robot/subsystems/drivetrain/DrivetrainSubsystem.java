package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.modules.SimGyro;
import frc.robot.subsystems.drivetrain.modules.SimModule;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public class DrivetrainSubsystem extends SubsystemBase {
  private static final double BUMPER_WIDTH = 0.87;   
  private static final double BUMPER_LENGTH = 0.97;  
  private static final double MASS = 51.0;           
  private static final double INERTIA = 6.883;       
  private static final double WHEEL_RADIUS = 0.05;   
  private static final double FRICTION_COEFF = 1.19; 
  private static final double CURRENT_LIMIT_AMPS = 6.0;

  private static final double HALF_WHEELBASE = 0.331;
  private static final double HALF_TRACKWIDTH = 0.268;

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(HALF_WHEELBASE, HALF_TRACKWIDTH),    // front-left
    new Translation2d(HALF_WHEELBASE, -HALF_TRACKWIDTH),   // front-right
    new Translation2d(-HALF_WHEELBASE, HALF_TRACKWIDTH),   // back-left
    new Translation2d(-HALF_WHEELBASE, -HALF_TRACKWIDTH)   // back-right
  );

  private final SimModule[] modules = new SimModule[4];
  private final SimGyro gyro = new SimGyro();
  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field = new Field2d();

  private boolean simEnabled = false;
  private double lastTimestamp = Timer.getFPGATimestamp();

  public DrivetrainSubsystem() {
    modules[0] = new SimModule(new Translation2d(HALF_WHEELBASE, HALF_TRACKWIDTH), WHEEL_RADIUS);
    modules[1] = new SimModule(new Translation2d(HALF_WHEELBASE, -HALF_TRACKWIDTH), WHEEL_RADIUS);
    modules[2] = new SimModule(new Translation2d(-HALF_WHEELBASE, HALF_TRACKWIDTH), WHEEL_RADIUS);
    modules[3] = new SimModule(new Translation2d(-HALF_WHEELBASE, -HALF_TRACKWIDTH), WHEEL_RADIUS);

    poseEstimator = new SwerveDrivePoseEstimator(
      kinematics,
      gyro.getRotation2d(),
      getModulePositions(),
      new Pose2d()
    );
    field.setRobotPose(getPose());
  }

  public void drive(double xSpeed, double ySpeed, double rotRadiansPerSec, boolean fieldRelative) {
    ChassisSpeeds chassis;
    if (fieldRelative) {
      chassis = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotRadiansPerSec, gyro.getRotation2d());
    } else {
      chassis = new ChassisSpeeds(xSpeed, ySpeed, rotRadiansPerSec);
    }
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassis);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, getMaxPhysicalSpeed());
    setModuleStates(states);
  }

  public void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(states[i]);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] out = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) out[i] = modules[i].getState();
    return out;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] out = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) out[i] = modules[i].getPosition();
    return out;
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation2d() {
    return gyro.getRotation2d();
  }

  public void setSimEnabled(boolean enable) {
    simEnabled = enable;
    lastTimestamp = Timer.getFPGATimestamp();
  }

  public void stop() {
    for (SimModule m : modules) {
      m.setDesiredState(new SwerveModuleState(0, m.getState().angle));
    }
  }

  private double getMaxPhysicalSpeed() {
    return 5.0;
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTimestamp;
    lastTimestamp = now;

    if (simEnabled && dt > 0.0) {
      simulate(dt);
    }

    poseEstimator.update(gyro.getRotation2d(), getModulePositions());
    field.setRobotPose(getPose());
  }

  private void simulate(double dt) {
    SwerveModuleState[] states = getModuleStates();
    ChassisSpeeds chassis = kinematics.toChassisSpeeds(states);

    double maxLinearAccel = FRICTION_COEFF * 9.81; // m/s^2 
    for (SimModule m : modules) m.simulationPeriodic(dt);

    gyro.simulate(dt, chassis.omegaRadiansPerSecond);
  }
}
