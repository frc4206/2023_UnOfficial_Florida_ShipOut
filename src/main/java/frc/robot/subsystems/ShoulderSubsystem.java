// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShoulderSubsystem extends SubsystemBase {
  public static CANSparkMax shoulderMotor = new CANSparkMax(Constants.Arm.Shoulder.shoulderMotorID, MotorType.kBrushless);
  public static RelativeEncoder shoulderenc = shoulderMotor.getEncoder();
  public static AbsoluteEncoder shoulderabsEncoder = shoulderMotor.getAbsoluteEncoder(Type.kDutyCycle);
  public static SparkMaxPIDController spidController;
  DigitalInput shoulderlimSwitch = new DigitalInput(6);

  public static double shoulderkP;//.0007473
  public static double shoulderkI;
  public static double shoulderkD;
  public static double shouldermaxVelo = 4000;
  public static double shoulderminVelo = 0;
  public static double shouldermaxAcc = 100;
  public static double shoulderallowedErr = 0.0005;
  public static double shoulderampLimit = 60;
  public static double shouldertargetPosition = 0.5;
  public static double shoulderupperLimit = 0.75;
  public static double shoulderLowerLimit = 0.001;
  public static boolean shoulderSafeToRun = true;

  
  boolean init = false;

  public ShoulderSubsystem() {
    shoulderkP = 0.022;
    shoulderkI = 0;
    shoulderkD = 0.0005;
    shoulderMotor.restoreFactoryDefaults();
    shoulderMotor.setIdleMode(IdleMode.kBrake);
    shoulderMotor.setInverted(true);
    shoulderMotor.setSmartCurrentLimit((int)Constants.Arm.Shoulder.ampLimit); //max currrent rating not exceed 60A or 100A more than 2 sec

    shoulderenc = shoulderMotor.getEncoder();
    spidController = shoulderMotor.getPIDController();
    //shoulderenc.setPosition(0.0);
    spidController.setFeedbackDevice(shoulderabsEncoder);

    spidController.setP(shoulderkP);
    spidController.setI(shoulderkI);
    spidController.setD(shoulderkD);
    spidController.setIZone(0);
    spidController.setFF(.000156);
    spidController.setOutputRange(-0.75, 0.75);
    spidController.setSmartMotionMaxVelocity(shouldermaxVelo, 0);
    spidController.setSmartMotionMinOutputVelocity(shoulderminVelo, 0);
    spidController.setSmartMotionMaxAccel(shouldermaxAcc, 0);
    spidController.setSmartMotionAllowedClosedLoopError(shoulderallowedErr, 0);
  }

  public void TestshoulderUp(/*Axis axis*/){
    shoulderMotor.set(0.7);
  }

  public void TestshoulderDown(/*Axis axis*/){
    shoulderMotor.set(-0.7);
  }

  public void shoulderStop(){
    shoulderMotor.set(0);
  }

  public void shoulderGoToZero(){
    if (shoulderlimSwitch.get()){
      shoulderMotor.set(-0.3);
    } else {
      shoulderMotor.set(0);
    }
  }

  public double shoulderPosition() {
    return shoulderabsEncoder.getPosition();
  }
  public double shoulderAllowedErr() {
    return shoulderallowedErr;
  }
  public double shoulderAmps() {
    return shoulderMotor.getOutputCurrent();
  }

  public boolean goToPositionShoulder(double setPosition){
    Constants.Arm.Shoulder.desiredPos = setPosition;
    double currPosShoulder = shoulderabsEncoder.getPosition();
    double currPosElevator = SmartDashboard.getNumber("Elevator Encoder", 0);

    if (currPosShoulder > setPosition-shoulderallowedErr && currPosShoulder < setPosition+shoulderallowedErr){
      shoulderMotor.set(0);
      return false;
    }

    
    if (currPosShoulder >= shoulderupperLimit) {
      shoulderSafeToRun = false;
    }
    if (currPosElevator < 30.0 && currPosShoulder <= .17) {
      shoulderSafeToRun = false;
    } else {
      shoulderSafeToRun = true;
    }
    if (currPosShoulder <= shoulderLowerLimit) {
      shoulderSafeToRun = false;
    }

    
     
    /* else if (shoulderMotor.get() < 0) {
      if (currPosShoulder <= shoulderLowerLimit) {
        shoulderSafeToRun = false;
      } else {
        if (currPosElevator < 37.0) {
          shoulderSafeToRun = false;
        } else {
          shoulderSafeToRun = true;
        }
      }
    }*/
    


    if (shoulderSafeToRun) {
      spidController.setReference(setPosition, CANSparkMax.ControlType.kSmartMotion);
      return true;
    } else {
      shoulderMotor.set(0);
      return false;
    }
  }

  public void init(){
    if (init == false){
      /*SmartDashboard.putNumber("shoulder kP", shoulderkP);
      SmartDashboard.putNumber("shoulder kI", shoulderkI);
      SmartDashboard.putNumber("shoulder kD", shoulderkD);
      SmartDashboard.putNumber("shoulder max Velo", shouldermaxVelo);
      SmartDashboard.putNumber("shoulder min Velo", shoulderminVelo);
      SmartDashboard.putNumber("shoulder max Acc", shouldermaxAcc);
      SmartDashboard.putNumber("shoulder Allowed Error", shoulderallowedErr);
      SmartDashboard.putNumber("shoulder Target Position", shouldertargetPosition);
      SmartDashboard.putNumber("shoulder Amp Limit", shoulderampLimit);*/
      init = true;
    }
  }





  @Override
  public void periodic() {
    init();
    GlobalVariables.shoulderabsEncoder = shoulderabsEncoder.getPosition();
    SmartDashboard.putNumber("Shoulder Encoder", shoulderabsEncoder.getPosition());
    SmartDashboard.putBoolean("Shoulder Safe To Run", shoulderSafeToRun);
    SmartDashboard.putBoolean("shoulder limit switch", shoulderlimSwitch.get());

    /*double repshoulderkP;
    double repshoulderkI;
    double repshoulderkD;
    double repshouldermaxVelo;
    double repshoulderminVelo;
    double repshouldermaxAcc;
    double repshoulderallowedErr;
    double repshoulderPosition;
    double repshoulderampLimit;

    repshoulderkP = SmartDashboard.getNumber("shoulder kP", 0.01);
    repshoulderkI = SmartDashboard.getNumber("shoulder kI", 0.01);
    repshoulderkD = SmartDashboard.getNumber("shoulder kD", 0.01);
    repshouldermaxVelo = SmartDashboard.getNumber("shoulder max Velo", 2000);
    repshoulderminVelo = SmartDashboard.getNumber("shoulder min Velo", 0);
    repshouldermaxAcc = SmartDashboard.getNumber("shoulder max Acc", 100);
    repshoulderallowedErr = SmartDashboard.getNumber("shoulder Allowed Error", 5);
    repshoulderPosition = SmartDashboard.getNumber("shoulder Target Position", 1000);
    repshoulderampLimit = SmartDashboard.getNumber("shoulder Amp Limit", 60);

    shoulderkP = repshoulderkP;
    shoulderkI = repshoulderkI;
    shoulderkD= repshoulderkD;
    shouldermaxVelo = repshouldermaxVelo;
    shoulderminVelo = repshoulderminVelo;
    shouldermaxAcc = repshouldermaxAcc;
    shoulderallowedErr = repshoulderallowedErr;
    shouldertargetPosition = repshoulderPosition;
    shoulderampLimit = repshoulderampLimit;*/
  }
}
