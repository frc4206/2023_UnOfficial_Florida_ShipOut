// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.GlobalVariables;

public class ElevatorSubsystem extends SubsystemBase {
  public static CANSparkMax elevatorMotor = new CANSparkMax(Constants.Arm.Elevator.elevatorMotorID, MotorType.kBrushless);
  public static RelativeEncoder elevatorenc = elevatorMotor.getEncoder();
  public static SparkMaxPIDController epidController;
  public static SparkMaxPIDController spidController;
  public static SparkMaxPIDController wpidController;
  //DigitalInput beamBreak = new DigitalInput(10);
  public DigitalInput lowlimSwitch = new DigitalInput(7);
  public DigitalInput highlimSwitch = new DigitalInput(9);
  double encPositionE = elevatorenc.getPosition();
  boolean innit = false;
  boolean elevatorMotorInnit;

  double manualPower = 0.0;

  public static double elevatorkP;//.0007473
  public static double elevatorkI;
  public static double elevatorkD;
  public static double elevatormaxVelo = 5000;
  public static double elevatorminVelo = 0;
  public static double elevatormaxAcc = 2000;
  public static double elevatorallowedErr = 2;
  public static double elevatorampLimit = 60;
  public static boolean elevatorSafeToRun = true;

  //PID
  public ElevatorSubsystem() {
  elevatorMotorInnit = false;
  /*----------------------------elevator--------------------------------------------- */
  double elevatorkP = .000035;//.0012
  double elevatorkI = 0;
  double elevatorkD = 0.01;
  elevatorMotor.restoreFactoryDefaults();
  elevatorMotor.setIdleMode(IdleMode.kBrake);
  elevatorMotor.setInverted(true);
  elevatorMotor.setSmartCurrentLimit((int)Constants.Arm.Elevator.ampLimit); //max currrent rating not exceed 60A or 100A more than 2 sec


  elevatorenc = elevatorMotor.getEncoder();
  epidController = elevatorMotor.getPIDController();
  epidController.setFeedbackDevice(elevatorenc);

  epidController.setP(elevatorkP);
  epidController.setI(elevatorkI);
  epidController.setD(elevatorkD);
  epidController.setIZone(0);
  epidController.setFF(.000156);
  epidController.setOutputRange(-0.75, 0.75);
  epidController.setSmartMotionMaxVelocity(elevatormaxVelo, 0);
  epidController.setSmartMotionMinOutputVelocity(elevatorminVelo, 0);
  epidController.setSmartMotionMaxAccel(elevatormaxAcc, 0);
  epidController.setSmartMotionAllowedClosedLoopError(elevatorallowedErr, 0);
  }



  //Temporary until I find actual positions for specific arm positions
  public void TestElevatorUp(){
    elevatorMotor.set(0.2);
  }

  public void TestElevatorDown(){
    elevatorMotor.set(-0.2);
  }

  public void UpFastForward(){
    elevatorMotor.set(0.55);
  }

  public void UpFastReverse(){
    elevatorMotor.set(-0.55);
  }

  public void UpSlowForward(){
    elevatorMotor.set(0.2);
  }

  public void UpSlowReverse(){
    elevatorMotor.set(-0.2);
  }

  public void resetLinkage(CANSparkMax motor) {
    motor.set(0);
    motor.getEncoder().setPosition(0.0);
  }

  public void InitalizeLinkage(DigitalInput i, CANSparkMax motor){
    if (i.get()){
      resetLinkage(motor);
    } else {
      motor.set(-0.2);
    }
  }

  public boolean checkIfHasSpace (double currentPos, double crosspoint, boolean greater) {
    if (greater) {
      if (currentPos > crosspoint) {
        return true;
      } else  {
        return false;
      }
    } else {
      if (currentPos < crosspoint) {
        return true;
      } else  {
        return false;
      }
    }
  }

  public void elevatorStop(){
    elevatorMotor.set(0);
  }

  public void hitSwitch () {
    if (lowlimSwitch.get() && elevatorMotor.get() < 0) {
      elevatorMotor.set(0);
      elevatorMotor.getEncoder().setPosition(0.0);
    }
  } 

  public boolean ishitSwitch () {
    return lowlimSwitch.get();
  } 

  public void resetElev () {
    elevatorMotor.set(0);
    elevatorMotor.getEncoder().setPosition(0.0);
  }

  public void goToZero() {
    if (lowlimSwitch.get()) {
      elevatorMotor.set(-0.4);
    } else {
      elevatorMotor.set(0.0);
      elevatorMotor.getEncoder().setPosition(0.0);
    }
  }

  public void goToTop(){
    if (highlimSwitch.get()){
      elevatorMotor.set(0.4);
    } else {
      elevatorMotor.set(0);
    }
  }

  public void goToPositionElevator(double setpoint) {   
    if (elevatorenc.getPosition() > setpoint-elevatorallowedErr && elevatorenc.getPosition() < setpoint+elevatorallowedErr){
      elevatorMotor.set(0);
      return;
    }
    
    if (setpoint < 30) {
      if (Constants.Arm.Shoulder.desiredPos < .18) {
        if (SmartDashboard.getNumber("Shoulder Encoder", 0) > .17) {
          if (elevatorenc.getPosition() < 30) {
            elevatorSafeToRun = false;
          } else {
            elevatorSafeToRun = true;
          }
        }
      } else {
        elevatorSafeToRun = true;
      }
    } else {
      elevatorSafeToRun = true;
    }

    if (elevatorSafeToRun) {
      epidController.setReference(setpoint, CANSparkMax.ControlType.kSmartMotion);
    } else {
      elevatorMotor.set(0);
    }
    
  }
  

  public void innit() {
    if (innit == false) {
      /*SmartDashboard.putNumber("elevator kP", elevatorkP);
      SmartDashboard.putNumber("elevator kI", elevatorkI);
      SmartDashboard.putNumber("elevator kD", elevatorkD);
      SmartDashboard.putNumber("elevator max Velo", elevatormaxVelo);
      SmartDashboard.putNumber("elevator min Velo", elevatorminVelo);
      SmartDashboard.putNumber("elevator max Acc", elevatormaxAcc);
      SmartDashboard.putNumber("elevator Allowed Error", elevatorallowedErr);
      SmartDashboard.putNumber("elevator Amp Limit", elevatorampLimit);*/
      innit = true;
    }
  }

  
  //------------------------------------------------------------------

  @Override
  public void periodic() {
    GlobalVariables.elevatorenc = elevatorenc.getPosition();
    SmartDashboard.putNumber("Elevator Encoder", elevatorenc.getPosition());
    SmartDashboard.putBoolean("Elevator Safe To Run", elevatorSafeToRun);

    innit();

    /*double repelevatorkP;
    double repelevatorkI;
    double repelevatorkD;
    double repelevatormaxVelo;
    double repelevatorminVelo;
    double repelevatormaxAcc;
    double repelevatorallowedErr;
    double repelevatorampLimit;

    repelevatorkP = SmartDashboard.getNumber("elevator kP", 0.01);
    repelevatorkI = SmartDashboard.getNumber("elevator kI", 0.01);
    repelevatorkD = SmartDashboard.getNumber("elevator kD", 0.01);
    repelevatormaxVelo = SmartDashboard.getNumber("elevator max Velo", 2000);
    repelevatorminVelo = SmartDashboard.getNumber("elevator min Velo", 0);
    repelevatormaxAcc = SmartDashboard.getNumber("elevator max Acc", 100);
    repelevatorallowedErr = SmartDashboard.getNumber("elevator Allowed Error", 5);
    repelevatorampLimit = SmartDashboard.getNumber("elevator Amp Limit", 60);


    elevatorkP = repelevatorkP;
    elevatorkI = repelevatorkI;
    elevatorkD= repelevatorkD;
    elevatormaxVelo = repelevatormaxVelo;
    elevatorminVelo = repelevatorminVelo;
    elevatormaxAcc = repelevatormaxAcc;
    elevatorallowedErr = repelevatorallowedErr;
    elevatorampLimit = repelevatorampLimit;*/
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Rev Encoder", altEncoder.getPosition());

    // getLastError value zero is no error
    //SmartDashboard.putNumber("Elevator Neo Encoder", elevatorMotor.getEncoder().getPosition());
    /*SmartDashboard.putNumber("Elevator Neo Speed", elevatorMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Elevator Neo Amps", elevatorMotor.getOutputCurrent());
    SmartDashboard.putNumber("Elevator Neo Temp", elevatorMotor.getMotorTemperature());*/
    
    if (elevatorMotor.getLastError().value == 0) {
      SmartDashboard.putBoolean("Elevator Neo Status", true);
    } else {
      SmartDashboard.putBoolean("Elevator Neo Status", false);
    }
    SmartDashboard.putBoolean("low limit switch", lowlimSwitch.get());
    SmartDashboard.putBoolean("high limit switch ", highlimSwitch.get());
    //SmartDashboard.putBoolean("beam", beamBreak.get());
  }
}
