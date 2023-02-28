// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class ResetOdometryCommand extends CommandBase {
  /** Creates a new ResetOdometryCommand. */
  SwerveSubsystem m_swerve;
  Rotation2d noRotation2d = new Rotation2d(0,0);
  Pose2d noPose2d = new Pose2d(0,0, noRotation2d);
  boolean fisnished = false;
  public ResetOdometryCommand(SwerveSubsystem swerve) {
    m_swerve = swerve;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.resetOdometry(noPose2d);
    fisnished = true;
    isFinished();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return fisnished;
  }
}
