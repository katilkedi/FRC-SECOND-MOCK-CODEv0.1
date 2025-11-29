package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.constants.OIConstants;
import frc.robot.subsystems.drivetrain.DrivetrainSubsystem;
import frc.robot.subsystems.drivetrain.modules.SimGyro;
import frc.robot.subsystems.drivetrain.modules.SimModule;

public class RobotContainer {

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final PS5Controller driverController = new PS5Controller(OIConstants.kDriverControllerPort);
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    configureBindings();

    // Teleop sürüş komutu
    drivetrainSubsystem.setDefaultCommand(
      new RunCommand(
        () -> drivetrainSubsystem.drive(
          -driverController.getLeftY(),
          -driverController.getLeftX(),
          -driverController.getRightX(),
          true
        ),
        drivetrainSubsystem
      )
    );

    configureAutonomous();
  }

  private void configureBindings() {
    // İleride buton atamaları buraya eklenebilir
  }

  private void configureAutonomous() {
    autoChooser.setDefaultOption("Simple Forward and Stop", getAutonomousCommand());
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  public void realSetup() {
    drivetrainSubsystem.setSimEnabled(false);
  }

  public void simSetup() {
    drivetrainSubsystem.setSimEnabled(true);
  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      new RunCommand(
        () -> drivetrainSubsystem.drive(0.5, 0.0, 0.0, true),
        drivetrainSubsystem
      ).withTimeout(2.0),
      new InstantCommand(() -> drivetrainSubsystem.stop(), drivetrainSubsystem)
    );
  }

  public Command getSelectedAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public DrivetrainSubsystem getDrivetrainSubsystem() {
    return drivetrainSubsystem;
  }
}
