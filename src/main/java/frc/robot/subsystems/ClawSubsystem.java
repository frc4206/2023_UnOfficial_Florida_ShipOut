// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
  public static CANSparkMax clawMotor = new CANSparkMax(Constants.Arm.Claw.clawMotorID, MotorType.kBrushless); 
  
  // Ultrasonic Sensor plugged into ANALOG IN port number 0
  private AnalogInput ultraSensor = new AnalogInput(0);
  
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

  public ClawSubsystem() {
    clawMotor.setSmartCurrentLimit((int)Constants.Arm.Claw.ampLimit); //max currrent rating not exceed 60A or 100A more than 2 sec

  }

  public void TestclawUp(){
    clawMotor.set(0.8);
  }

  public void TestclawDown(){
    clawMotor.set(-0.8);
  }

  public void clawStop(){
    clawMotor.set(0);
  }

  public double clawAmps() {
    return clawMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Color detectedColor = colorSensor.getColor();
    SmartDashboard.putNumber("red", detectedColor.red);
    SmartDashboard.putNumber("blue", detectedColor.blue);
    SmartDashboard.putNumber("green", detectedColor.green);
  }
}
