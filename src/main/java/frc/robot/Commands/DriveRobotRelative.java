// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
public class DriveRobotRelative extends Command {
  /** Creates a new DriveFieldRelative. */
  CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  double MaxAngularRate = 1.5 * Math.PI;
  SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  double xSpeeda;
  double ySpeeda;
  double thetaSpeeda;
  public DriveRobotRelative(double xSpeed, double ySpeed, double thetaSpeed) {
    xSpeeda = xSpeed;
    ySpeeda = ySpeed;
    thetaSpeeda = thetaSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.applyRequest(() -> drive.withVelocityX(xSpeeda * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(ySpeeda * MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(thetaSpeeda * MaxAngularRate) // Drive counterclockwise with negative X (left)
        );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
