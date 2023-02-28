  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

public class PID_Distance extends CommandBase {
  Limelight Photonvision;
  SwerveSubsystem swerveSubsystem;
  Joystick controller;
  boolean isfinished; 

  private double rotation;

  private int strafeAxis;
  private int rotationAxis;
  private int translationAxis;

  private Translation2d translation;
  private boolean fieldRelative;
  private boolean openLoop;
  
  double StartCommandTime;
  double CurrentTime;
  
  public double lastErrorX = 0;
  public double lastErrorY = 0;
  public double lastErrorYaw = 0;

  public double xSet;
  public double ySet;
  public double yawSet;
  public double timeout;

  /** Creates a new PID_Distance. */
  public PID_Distance(Limelight m_photonVision, SwerveSubsystem m_swerveSubsystem, Joystick m_controller, int m_translationAxis, int m_strafeAxis, int m_rotationAxis, boolean m_fieldRelative, boolean m_openLoop, double m_xSetpoint, double m_ySetpoint, double m_yawSetpoint, double m_timeout) {
    Photonvision = m_photonVision;
    swerveSubsystem = m_swerveSubsystem;
    addRequirements(Photonvision);
    addRequirements(swerveSubsystem);

    xSet = m_xSetpoint;
    ySet = m_ySetpoint;
    yawSet = m_yawSetpoint;
    timeout = m_timeout;

    controller = m_controller;


    translationAxis = m_translationAxis;
    strafeAxis = m_strafeAxis;
    rotationAxis = m_rotationAxis;
    fieldRelative = m_fieldRelative;
    openLoop = m_openLoop;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isfinished = false;
    StartCommandTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CurrentTime = Timer.getFPGATimestamp() - StartCommandTime;
    double m_kP = Limelight.XkP;
    double m_kI = Limelight.XkI;
    double m_kD = Limelight.XkD;
    double m_YkP = Limelight.YkP;
    double m_YkD = Limelight.YkD;
    double m_YkI = Limelight.YkI;
    double m_YawkP = Limelight.YawkP;
    double m_YawkI = Limelight.YawkI;
    double m_YawkD = Limelight.YawkD;
    double m_MaxError = .05038;
    double[] robotToCam = {.1143, .00762};

    //0.57
    double errorX = Photonvision.X - xSet + robotToCam[0];
    double errorDifferenceX = errorX - lastErrorX;
    double outputX = errorX*m_kP + errorDifferenceX*m_kD;

    double errorY = Photonvision.Y - ySet + robotToCam[1];
    double errorDifferenceY = errorY - lastErrorY;
    double errorSumY = errorY += errorY;
    double outputY = errorY*m_YkP + errorDifferenceY*m_YkD + errorSumY*m_YkI;

    if (errorY > .5) {
      outputY = errorY*m_YkP + errorDifferenceY*m_YkD;
    } else {
      outputY = errorY*m_YkP + errorDifferenceY*m_YkD + errorSumY*m_YkI;
    }
    
    double errorYaw = Photonvision.Yaw - yawSet;
    double errorDifferenceYaw = errorYaw - lastErrorYaw;
    double errorSumYaw = errorYaw += errorYaw;
    double outputYaw = errorYaw*m_YawkP + errorDifferenceYaw*m_YawkD + errorSumYaw*m_YawkI;
    
    double yAxis = outputX;
    double xAxis = outputY;
    double rAxis = -outputYaw;
    
    //(Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis
    //xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
    //rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;
    
    translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
    rotation = rAxis * Constants.Swerve.maxAngularVelocity;
    swerveSubsystem.drive(translation, rotation, fieldRelative, openLoop);

    lastErrorX = errorX;
    lastErrorY = errorY;
    lastErrorYaw = errorYaw;

    //0.05038
    if (Math.abs(errorX) < m_MaxError && Math.abs(errorY) < m_MaxError && Math.abs(errorYaw) < m_MaxError) {
      System.out.println("terminated");

      isfinished = true;
      isFinished();
    }
    if (CurrentTime > timeout) {
      System.out.println("terminated on timeout");
      System.out.println(errorX);
      System.out.println(errorY);
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
