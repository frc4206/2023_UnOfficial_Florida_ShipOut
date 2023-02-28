// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class SubstationATCommand extends CommandBase {
  Limelight m_PhotonVision;
  SwerveSubsystem m_SwerveSubsystem;
  Joystick m_controller;
  boolean isfinished; 



  private double rotation;
  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;
  
  private int translationAxis;
  private int strafeAxis;
  private int rotationAxis;
  
  public double lastErrorX = 0;
  public double lastErrorY = 0;

  /** Creates a new PID_Distance. */
  public SubstationATCommand(Limelight photonVision, SwerveSubsystem swerveSubsystem, Joystick controller, int m_translationAxis, int m_strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
    m_PhotonVision = photonVision;
    m_SwerveSubsystem = swerveSubsystem;
    addRequirements(m_PhotonVision);
    addRequirements(m_SwerveSubsystem);

    m_controller = controller;


    this.translationAxis = translationAxis;
    this.strafeAxis = strafeAxis;
    this.rotationAxis = rotationAxis;
    this.fieldRelative = fieldRelative;
    this.openLoop = openLoop;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isfinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double m_kP = 0.125;
    double m_kD = 0.001;

    double m_YkP = 0.10625;
    double m_YkD = 0.006;

    //1.04 for y
    double errorX = m_PhotonVision.X + 0.94;
    double errorDifferenceX = errorX - lastErrorX;
    double outputX = errorX*m_kP + errorDifferenceX*m_kD;

    double errorY = m_PhotonVision.Y + 1.29;
    double errorDifferenceY = errorY - lastErrorY;
    double outputY = errorY*m_YkP + errorDifferenceY*m_YkD;
    //double outputY = new PIDController(m_YkP, 0, m_YkD).calculate(errorY, 0);

    
    double yAxis = outputX;
    double xAxis = outputY;
    double rAxis = -m_controller.getRawAxis(rotationAxis)*0.75;
    //-m_controller.getRawAxis(strafeAxis)*0.75;
    
    //(Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis
    xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
    rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;
    
    translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
    rotation = rAxis * Constants.Swerve.maxAngularVelocity;
    m_SwerveSubsystem.drive(translation, rotation, fieldRelative, openLoop);

    lastErrorX = errorX;
    lastErrorY = errorY;

    //0.05038
    if (Math.abs(errorX) < 0.1 && Math.abs(errorY) < 0.1) {
      System.out.println("terminated");
      isfinished = true;
      isFinished();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isfinished;
  }
}
