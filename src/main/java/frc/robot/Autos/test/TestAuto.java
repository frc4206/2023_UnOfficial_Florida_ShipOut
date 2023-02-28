// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autos.test;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto extends SequentialCommandGroup {
  PathPlannerTrajectory path = PathPlanner.loadPath("charge station test", new PathConstraints(2, 1));
  
  public TestAuto(SwerveSubsystem swerve) {
    var thetaController = new PIDController(
      Constants.AutoConstants.kPThetaController, 0, 0);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PPSwerveControllerCommand group = new PPSwerveControllerCommand(
      path, 
      swerve::getPose, 
      Constants.Swerve.swerveKinematics, 
      new PIDController(Constants.AutoConstants.kPXControllerfast, 0, 0),
      new PIDController(Constants.AutoConstants.kPYControllerfast, 0, 0),      
      thetaController, 
      swerve::setModuleStates,  
      swerve);
      
    addCommands(
      new InstantCommand(() -> swerve.resetOdometry(path.getInitialHolonomicPose()), swerve), 
      group
    );
  }
}

