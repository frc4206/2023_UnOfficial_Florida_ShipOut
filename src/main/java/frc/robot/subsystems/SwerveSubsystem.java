package frc.robot.subsystems;

import java.io.IOException;

import com.ctre.phoenix.sensors.Pigeon2;
import frc.robot.SwerveModule;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SwerveSubsystem extends SubsystemBase {
    public Limelight m_PhotonVision;
    public SwerveDriveOdometry swerveOdometry;
    public static SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    Rotation2d noRotation2d = new Rotation2d(0,0);
    Pose2d noPose2d = new Pose2d(0,0, noRotation2d);
    public double[] convertedCords = {0,0};
    public Pose2d AprilCords = new Pose2d(0,0, noRotation2d);
    public double currPercent = 1;
    
    public SwerveSubsystem() throws IOException {
        m_PhotonVision = new Limelight();
        gyro = new Pigeon2(Constants.Swerve.pigeonID, Constants.Canivore1);
        gyro.configFactoryDefault();
        zeroGyro();
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
        
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
        
    }
    
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), 
                    translation.getY(), 
                    rotation, 
                    getYaw()
                    )
                    : new ChassisSpeeds(
                        translation.getX(), 
                        translation.getY(), 
                        rotation)
                        );
                        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
                        
                                    for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }
    
    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    
    
    public void changePercent(){
        if (currPercent == 1){
            currPercent = 0.5;
        } else if (currPercent == 0.5){
            currPercent = 1;
        }
    }

    public Pose2d getPose(){
        return swerveOdometry.getPoseMeters();
    }
    
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public void resetOdometryAprilTag(){
        convertedCords = m_PhotonVision.getconvertCordinateSystems();
        Pose2d aprilCords = new Pose2d (convertedCords[0], convertedCords[1], noRotation2d);
        AprilCords = aprilCords;
        resetOdometry(aprilCords);
    }
    
    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }
    
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }
    
    public void zeroGyro(){
        gyro.setYaw(0);
    }
    
    public void setGyro(double degrees){
        gyro.setYaw(degrees);
    }

    public Rotation2d getYaw() {
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]);
    }
    
    //Charge Station AutoBalancing
    //-------------------------------------------------------------------------------------------------------
  
    public void BalanceBrake(){
        Translation2d translation = new Translation2d(0, 0.05).times(Constants.Swerve.maxSpeed);
        double rotation = 0;
        drive(translation, rotation, true, true);
    }

    public void AutoAutoBalance(){
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        double pitch = ypr[1];
        double roll = ypr[2];

        //0.08125 for heavy weight (kP)
        //0.075 (kF)
        double kP = 0.08;
        double kF = 0.02;

        double errorP = pitch - 3.5;
        double outputP = errorP*kP + kF;

        double errorR = roll - 3.5;
        double outputR = errorR*kP + kF;
        
        while (!(pitch < 3.5 && pitch > -3.5)){
            Translation2d translation = new Translation2d(0, outputP).times(Constants.Swerve.maxSpeed-4);
            double rotation = 0;
            drive(translation, rotation, false, true);
            return;
        }

        while (!(roll < 3.5 && roll > -3.5)) {
            Translation2d translation = new Translation2d((outputR), 0).times(Constants.Swerve.maxSpeed-4);
            double rotation = 0;
            drive(translation, rotation, false, true);    
            return;
        }

        drive(new Translation2d(0,0), 0, true, true);
    }

    public void AutoAutoBalanceFar(){
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        double pitch = ypr[1];
        double roll = ypr[2];

        //0.08125 for heavy weight (kP)
        //0.075 (kF)
        double kP = 0.08;
        double kF = 0.02;

        double errorP = pitch - 3.5;
        double outputP = errorP*kP + kF;

        double errorR = roll - 3.5;
        double outputR = errorR*kP + kF;
        
        while (!(pitch < 3.5 && pitch > -3.5)){
            Translation2d translation = new Translation2d(0, -outputP).times(Constants.Swerve.maxSpeed-4);
            double rotation = 0;
            drive(translation, rotation, false, true);
            return;
        }

        while (!(roll < 3.5 && roll > -3.5)) {
            Translation2d translation = new Translation2d((-outputR), 0).times(Constants.Swerve.maxSpeed-4);
            double rotation = 0;
            drive(translation, rotation, false, true);    
            return;
        }

        drive(new Translation2d(0,0), 0, true, true);
    }

    public void AutoBalanceClose(){
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        double pitch = ypr[1];
        double roll = ypr[2];

        double kP = 0.125;

        double errorP = Math.abs(pitch) - 8;
        double outputP = errorP*kP;

        double errorR = Math.abs(roll) - 8;
        double outputR = errorR*kP;
        
        if (!(pitch < 2 && pitch > -2)){
            Translation2d translation = new Translation2d(outputP, 0).times(Constants.Swerve.maxSpeed-2);
            double rotation = 0;
            drive(translation, rotation, true, true);    
            return;
        } 

        if (!(roll < 2 && roll > -2)) {
            Translation2d translation = new Translation2d(outputR, 0).times(Constants.Swerve.maxSpeed-2);
            double rotation = 0;
            drive(translation, rotation, true, true);    
            return;
        } 
        
        Translation2d translation = new Translation2d(0, 0).times(Constants.Swerve.maxSpeed-2);
        double rotation = 0;
        drive(translation, rotation, true, true);    
    }

    
    public void AutoBalanceFar(){
        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        double pitch = ypr[1];
        double roll = ypr[2];

        double kP = 0.15;

        double errorP = Math.abs(pitch) - 8;
        double outputP = errorP*kP;

        double errorR = Math.abs(roll) - 8;
        double outputR = errorR*kP;
        
        if (!(pitch < 2 && pitch > -2)){
            Translation2d translation = new Translation2d(-outputP, 0).times(Constants.Swerve.maxSpeed-2);
            double rotation = 0;
            drive(translation, rotation, true, true);    
            return;
        } 

        if (!(roll < 2 && roll > -2)) {
            Translation2d translation = new Translation2d(-outputR, 0).times(Constants.Swerve.maxSpeed-2);
            double rotation = 0;
            drive(translation, rotation, true, true);    
            return;
        } 
        
        Translation2d translation = new Translation2d(0, 0).times(Constants.Swerve.maxSpeed-2);
        double rotation = 0;
        drive(translation, rotation, true, true);    
    }
    //-------------------------------------------------------------------------------------------------------


    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());
        
        double[] OdometryArray = {getPose().getX(), getPose().getY()};
        SmartDashboard.putNumberArray("OdometryArray", OdometryArray);

        double[] convertCords = {AprilCords.getX(), AprilCords.getY()};
        SmartDashboard.putNumberArray("AprilConvertedCords", convertCords);

        double[] ypr = new double[3];
        gyro.getYawPitchRoll(ypr);
        SmartDashboard.putNumber("yaw", ypr[0]);
        SmartDashboard.putNumber("gyro angle", ypr[1]);
        SmartDashboard.putNumber("roll", ypr[2]);

        SmartDashboard.putNumber("currPercent", currPercent);

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            /*SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Drive Amps", mod.mDriveMotor.getSupplyCurrent());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle Amps", mod.mAngleMotor.getSupplyCurrent());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Drive Temp", mod.mDriveMotor.getTemperature());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle Temp", mod.mAngleMotor.getTemperature());
            if (mod.mDriveMotor.getLastError().value == 0) { 
              SmartDashboard.putBoolean("Mod " + mod.moduleNumber + " Drive Status", true);
            } else {
              SmartDashboard.putBoolean("Mod " + mod.moduleNumber + " Drive Status", false);
            }
            if (mod.mAngleMotor.getLastError().value == 0) { 
                SmartDashboard.putBoolean("Mod " + mod.moduleNumber + " Angle Status", true);
            } else {
                SmartDashboard.putBoolean("Mod " + mod.moduleNumber + " Angle Status", false);
            }*/
        }
    }
}