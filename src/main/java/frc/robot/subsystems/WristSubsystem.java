// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.GlobalVariables;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
  public static CANSparkMax wristMotor = new CANSparkMax(Constants.Arm.Wrist.wristMotorID, MotorType.kBrushless);
  public static CANSparkMax testMotor = new CANSparkMax(5, MotorType.kBrushed);
  public static AbsoluteEncoder wristabsEncoder = wristMotor.getAbsoluteEncoder(Type.kDutyCycle);
  public static SparkMaxPIDController wpidController;

  public static double wristkP = 0.005;//.0007473
  public static double wristkI = 0.0;
  public static double wristkD = 0.0;
  public static double wristmaxVelo = 1000;
  public static double wristminVelo = 0;
  public static double wristmaxAcc = 100;
  public static double wristallowedErr = 0.0005;
  public static double wristampLimit = 60;
  public static double wristtargetPosition = 0.3;
  public static double wristupperLimit = .63;
  public static double wristLowerLimit = .0001;
  public static boolean wristSafeToRun = false;

  public DigitalInput wristLimSwitch = new DigitalInput(8);

  boolean init = false;

  public WristSubsystem() {
    wristMotor.restoreFactoryDefaults();
    wristMotor.setIdleMode(IdleMode.kBrake);
    wristMotor.setInverted(false);
    wristMotor.setSmartCurrentLimit((int)Constants.Arm.Wrist.ampLimit); //max currrent rating not exceed 60A or 100A more than 2 sec

    wristabsEncoder.setInverted(false);

    wpidController = wristMotor.getPIDController();
    //wristenc.setPosition(0.0);
    wpidController.setFeedbackDevice(wristabsEncoder);

    wpidController.setP(wristkP);
    wpidController.setI(wristkI);
    wpidController.setD(wristkD);
    wpidController.setIZone(0);
    wpidController.setFF(.000156);
    wpidController.setOutputRange(-0.5, 0.5);
    wpidController.setSmartMotionMaxVelocity(wristmaxVelo, 0);
    wpidController.setSmartMotionMinOutputVelocity(wristminVelo, 0);
    wpidController.setSmartMotionMaxAccel(wristmaxAcc, 0);
    wpidController.setSmartMotionAllowedClosedLoopError(wristallowedErr, 0);
  }

  public void TestwristUp(/*Axis axis*/){
    wristMotor.set(0.3);
  }

  public void TestwristDown(/*Axis axis*/){
    wristMotor.set(-0.3);
  }

  public void wristStop(){
    wristMotor.set(0);
  }

  public void goToZero(){
    if (wristLimSwitch.get()){
      wristMotor.set(-0.3);
    } else {
      wristMotor.set(0);
    }
  }


  public boolean goToPositionWrist(double setPosition){
    double currPosWrist = wristabsEncoder.getPosition();
    double currPosElevator = GlobalVariables.elevatorenc;

    if (currPosWrist > setPosition-wristallowedErr && currPosWrist < setPosition+wristallowedErr){
      wristMotor.set(0);
      return true;
    }

    if (wristMotor.get() < 0) {
      if (currPosWrist == wristupperLimit || currPosWrist > .9) {
        wristSafeToRun = false;
      } else {
        wristSafeToRun = true;
      }
    } else if (wristMotor.get() > 0) {
      if (currPosWrist < wristLowerLimit+0.02 && currPosWrist > wristLowerLimit-0.05) {
        wristSafeToRun = false;
      } else {
        if (currPosElevator < 37 && GlobalVariables.shoulderabsEncoder < .178) {
          wristSafeToRun = false;
        } else if (currPosElevator >= 37 && GlobalVariables.shoulderabsEncoder >= .178){
          wristSafeToRun = true;
        } else {
          wristSafeToRun = false;
        }
      }
    }
    

    if (currPosElevator < 30 && SmartDashboard.getNumber("Shoulder Encoder", 0) < .178) {
      wristSafeToRun = false;
    } else {
      wristSafeToRun = true;
    }

  
    if (wristSafeToRun) {
      wpidController.setReference(setPosition, CANSparkMax.ControlType.kSmartMotion);
      return true;
    } else {
      wristMotor.set(0);
      return false;
    }
  }

  public void init(){
    if (init == false){
      /*SmartDashboard.putNumber("wrist kP", wristkP);
      SmartDashboard.putNumber("wrist kI", wristkI);
      SmartDashboard.putNumber("wrist kD", wristkD);
      SmartDashboard.putNumber("wrist max Velo", wristmaxVelo);
      SmartDashboard.putNumber("wrist min Velo", wristminVelo);
      SmartDashboard.putNumber("wrist max Acc", wristmaxAcc);
      SmartDashboard.putNumber("wrist Allowed Error", wristallowedErr);
      SmartDashboard.putNumber("wrist Target Position", wristtargetPosition);
      SmartDashboard.putNumber("wrist Amp Limit", wristampLimit);*/
      init = true;
    }

  }





  @Override
  public void periodic() {
    double WristAmps = wristMotor.getOutputCurrent();
    double TestAmps = testMotor.getOutputCurrent();

    init();
    GlobalVariables.wristabsEncoder = wristabsEncoder.getPosition();
    SmartDashboard.putBoolean("Wrist Safe To Run", wristSafeToRun);
    SmartDashboard.putNumber("wrist encoder", wristabsEncoder.getPosition());

    /*double repwristkP;
    double repwristkI;
    double repwristkD;
    double repwristmaxVelo;
    double repwristminVelo;
    double repwristmaxAcc;
    double repwristallowedErr;
    double repwristPosition;
    double repwristampLimit;

    repwristkP = SmartDashboard.getNumber("wrist kP", 0.01);
    repwristkI = SmartDashboard.getNumber("wrist kI", 0.01);
    repwristkD = SmartDashboard.getNumber("wrist kD", 0.01);
    repwristmaxVelo = SmartDashboard.getNumber("wrist max Velo", 2000);
    repwristminVelo = SmartDashboard.getNumber("wrist min Velo", 0);
    repwristmaxAcc = SmartDashboard.getNumber("wrist max Acc", 100);
    repwristallowedErr = SmartDashboard.getNumber("wrist Allowed Error", 5);
    repwristPosition = SmartDashboard.getNumber("wrist Target Position", 1000);
    repwristampLimit = SmartDashboard.getNumber("wrist Amp Limit", 60);

    wristkP = repwristkP;
    wristkI = repwristkI;
    wristkD= repwristkD;
    wristmaxVelo = repwristmaxVelo;
    wristminVelo = repwristminVelo;
    wristmaxAcc = repwristmaxAcc;
    wristallowedErr = repwristallowedErr;
    wristtargetPosition = repwristPosition;
    wristampLimit = repwristampLimit;*/

    SmartDashboard.putBoolean("wrist limit switch", wristLimSwitch.get());
    //SmartDashboard.putNumber("Wrist Amps", WristAmps);
    //SmartDashboard.putNumber("Test Amps", TestAmps);
  }
}
