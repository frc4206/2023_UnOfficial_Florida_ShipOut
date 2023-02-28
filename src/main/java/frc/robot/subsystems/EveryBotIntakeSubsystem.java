// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EveryBotIntakeSubsystem extends SubsystemBase {
  public static CANSparkMax ebotmotor = new CANSparkMax(Constants.Intake.ebotmotorID, MotorType.kBrushless);
  /** Creates a new EveryBotIntakeSubsystem. */
  public EveryBotIntakeSubsystem() {
    //don't let motor stall after intake
    ebotmotor.restoreFactoryDefaults();
    ebotmotor.setIdleMode(IdleMode.kBrake);
    ebotmotor.setInverted(false);
    ebotmotor.setSmartCurrentLimit(Constants.Intake.ebotAmpLimit);
  }

  public void In() {
    ebotmotor.set(0.5);
    if (ebotmotor.getOutputCurrent() > 60) {
      Stop();
    } else {
      ebotmotor.set(0.5);
    }
  }

  public void Out() {
    ebotmotor.set(-0.5);
  }

  public void Stop() {
    ebotmotor.set(0.0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
