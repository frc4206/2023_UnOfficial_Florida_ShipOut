// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake3Subsystem extends SubsystemBase {
  /** Creates a new Intake3Subsystem. */
  public CANSparkMax intakemotor = new CANSparkMax(Constants.Intake.intake3motorID, MotorType.kBrushless);

  public Intake3Subsystem() {
    intakemotor.restoreFactoryDefaults();
    intakemotor.setIdleMode(IdleMode.kBrake);
    intakemotor.setInverted(false);
    intakemotor.setSmartCurrentLimit(Constants.Intake.intake3AmpLimit);
  }

  public void In() {
    intakemotor.set(0.5);
  }

  public void Out() {
    intakemotor.set(-0.5);
  }

  public void Stop() {
    intakemotor.set(0.0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
