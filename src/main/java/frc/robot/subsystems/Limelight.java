// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.IOException;
import java.util.function.Supplier;

public class Limelight extends SubsystemBase  {
  /** Creates a new PhotonVision. */
  public static double currPipeline = 0;
  
  boolean innit = false;

  double camHeight = 20.5;
  double targetHeight = 12.75;
  double degrees = -20;
  public double setpointX = 1;

  public static double XkP = 0.09;
  public static double XkI = 0.05;
  public static double XkD = 0.01;

  public static double YkP = 0.09;
  public static double YkD = 0.01;
  public static double YkI = 0.011;

  public static double YawkP = 0;
  public static double YawkI = 0;
  public static double YawkD = 0;

  public double Y = 0;
  public double Z = 0;
  public double X = 0;
  public double Yaw = 0;
  double[] cords = {X, Y, Z};
  public int id;

  double CAMERA_HEIGHT_IN = camHeight;
  double TARGET_HEIGHT_IN = targetHeight;
  Supplier<Double> supplier = () -> 0.1;
  double range;

  // Angle between horizontal and the camera.
  double CAMERA_PITCH_DEGREES = Units.degreesToRadians(degrees);
  // How far from the target we want to be
  // Change this to match the name of your camera
  //SwerveDrivePoseEstimator sPoseEstimator = new SwerveDrivePoseEstimator(null, null, null, null);
  public NetworkTable limelighttable;
  public Limelight() throws IOException {
    //newPose2d = new Pose2d();
    limelighttable = NetworkTableInstance.getDefault().getTable("limelight");
    System.out.println("works here");
  }

  public void changePipeline() {
    if (currPipeline == 0){
      limelighttable.getEntry("pipeline").setNumber(1);
      currPipeline = 1;
    }
    else if (currPipeline == 1){
      limelighttable.getEntry("pipeline").setNumber(0);
      currPipeline = 0;
    }
  }

  public double[] getconvertCordinateSystems () {
    double x = X + 5;
    double y = Y + 5;
    double[] ConvertedCords = {x,y};
    return ConvertedCords;
  }

  public void innit () {
    if (innit == false) {
      SmartDashboard.putNumber("repcamheight", 8.75);
      SmartDashboard.putNumber("reptargetHeight", 24);
      SmartDashboard.putNumber("degrees", 12);
      SmartDashboard.putNumber("kP", 0.23);
      SmartDashboard.putNumber("kI", 0.05);
      SmartDashboard.putNumber("kD", .06);
      SmartDashboard.putNumber("YkP", 0.14);
      SmartDashboard.putNumber("YkI", 0.025);
      SmartDashboard.putNumber("YkD", 0.19);
      SmartDashboard.putNumber("YawkP", 0.00045);
      SmartDashboard.putNumber("YawkI", 0.0);
      SmartDashboard.putNumber("YawkD", 0.00003);
      SmartDashboard.putNumber("Set Point X", 1);
      innit = true;
    }
  }
  @Override
  public void periodic() {
    innit();
    //System.out.println(result);
    double rep1;
    double rep2;
    double rep3;

    double repkP;
    double repkI;
    double repkD;
    double repYkP;
    double repYkD;
    double repYkI;
    double repYawkP;
    double repYawkI;
    double repYawkD;
    double repsetX;

    
    rep1 = SmartDashboard.getNumber("repcamheight", 0);
    rep2 = SmartDashboard.getNumber("reptargetHeight", 0);
    rep3 = SmartDashboard.getNumber("degrees", 0);

    repkP = SmartDashboard.getNumber("kP", 0.23);
    repkI = SmartDashboard.getNumber("kI", 0.05);
    repkD = SmartDashboard.getNumber("kD", 0.06);

    repYkP = SmartDashboard.getNumber("YkP", 0.14);
    repYkD = SmartDashboard.getNumber("YkD", 0.025);
    repYkI = SmartDashboard.getNumber("YkI", 0.19);

    repYawkP = SmartDashboard.getNumber("YawkP", 0.14);
    repYawkI = SmartDashboard.getNumber("YawkD", 0.025);
    repYawkD = SmartDashboard.getNumber("YawkI", 0.19);

    repsetX = SmartDashboard.getNumber("Set Point X", 1);


    camHeight = rep1;
    targetHeight = rep2;
    degrees = rep3;
    CAMERA_HEIGHT_IN = camHeight;
    TARGET_HEIGHT_IN = targetHeight;
    CAMERA_PITCH_DEGREES = degrees;


    XkP = repkP;
    XkI = repkI;
    XkD = repkD;
    YkP = repYkP;
    YkD = repYkD;
    YkI = repYkI;
    YawkP = repYawkP;
    YawkI = repYawkI;
    YawkD = repYawkD;
    setpointX = repsetX;
    //if (result.hasTargets()) {
      //range = PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT_METERS, TARGET_HEIGHT_METERS, CAMERA_PITCH_RADIANS, Units.degreesToRadians(result.getBestTarget().getPitch()));
    //}
    //boolean target = result.hasTargets();
    // This method will be called once per scheduler run
    //SmartDashboard.putBoolean("hasTarget", target);
    SmartDashboard.putNumber("range", range);

    
    //System.out.println("test");
    double[] result = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
    double result2 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    Y = result[0];
    Z = result[1];
    X = -result[2];
    Yaw = result2;
    cords[0] = X;
    cords[1] = Y;
    cords[2] = Z;
    
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
        
    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = CAMERA_PITCH_DEGREES;
        
    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = CAMERA_HEIGHT_IN;
        
    // distance from the target to the floor
    double goalHeightInches = TARGET_HEIGHT_IN;
        
    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
        
    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);

    SmartDashboard.putNumber("distance", distanceFromLimelightToGoalInches);
    SmartDashboard.putNumber("distance but hopefuly better", X);
    SmartDashboard.putNumberArray("cords", cords);
  }
}
