// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class AprilTagResetOdometryCommand extends CommandBase {
  /** Creates a new AprilTagResetOdometryCommand. */
  SwerveSubsystem m_swerve;
  boolean fisnished = false;
  public AprilTagResetOdometryCommand(SwerveSubsystem swerve) {
    m_swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.resetOdometryAprilTag();
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
