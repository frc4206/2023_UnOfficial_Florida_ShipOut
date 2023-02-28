// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Autos.BarrierSide.LinkBarrierSideBlue;
import frc.robot.Autos.OverDaStation.OverDaStationpt1;
import frc.robot.Autos.OverDaStation.OverDaStationpt2;
import frc.robot.Autos.WallSide.LinkWallSideBlue;
import frc.robot.Autos.test.TestAuto;
import frc.robot.commands.Combos.ArmRetrieveCube;
import frc.robot.commands.Combos.ArmTop;
import frc.robot.commands.Combos.ArmMiddle;
import frc.robot.commands.Combos.ArmMiddleCube;
import frc.robot.commands.Combos.ArmRetrieveCone;
import frc.robot.commands.Combos.ArmLowCone;
import frc.robot.commands.Combos.ArmLowCube;
import frc.robot.commands.Combos.ArmReturn;
import frc.robot.commands.Combos.ArmSingle;
import frc.robot.commands.Combos.ArmTest;
import frc.robot.commands.ChangeToLime;
import frc.robot.commands.Print_CommandTest;
import frc.robot.commands.Arm.Claw.ClawInCommand;
import frc.robot.commands.Arm.Claw.ClawOutCommand;
import frc.robot.commands.Arm.Elevator.ElevatorGoToPosition;
import frc.robot.commands.Arm.Elevator.ElevatorGoToTop;
import frc.robot.commands.Arm.Elevator.ElevatorGoToZero;
import frc.robot.commands.Arm.Elevator.ElevatorTestDownCommand;
import frc.robot.commands.Arm.Elevator.ElevatorTestUpCommand;
import frc.robot.commands.Arm.Elevator.ResetElevatorCommand;
import frc.robot.commands.Arm.Elevator.UpFastForward;
import frc.robot.commands.Arm.Elevator.UpFastReverse;
import frc.robot.commands.Arm.Shoulder.GotoPosShoulder;
import frc.robot.commands.Arm.Shoulder.TestShoulderDownCommand;
import frc.robot.commands.Arm.Shoulder.TestShoulderUpCommand;
import frc.robot.commands.Arm.Wrist.GoToPositionWrist;
import frc.robot.commands.Arm.Wrist.GoToZeroWrist;
import frc.robot.commands.Arm.Wrist.TestWristDownCommand;
import frc.robot.commands.Arm.Wrist.TestWristUpCommand;
import frc.robot.commands.Swerve.AutoAutoBalanceCommand;
import frc.robot.commands.Swerve.AutoAutoBalanceFarCommand;
import frc.robot.commands.Swerve.AutoBalanceCloseCommand;
import frc.robot.commands.Swerve.AutoBalanceFarCommand;
import frc.robot.commands.Swerve.BalanceBrakeCommand;
import frc.robot.commands.Swerve.HalfSpeed;
import frc.robot.commands.Swerve.PID_Distance;
import frc.robot.commands.Swerve.SubstationATCommand;
import frc.robot.commands.Swerve.TeleopSwerve;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.WristSubsystem;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  public static CommandXboxController driveController = new CommandXboxController(0);

  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);
  private final Joystick buttonBox = new Joystick(2);

  private final SendableChooser<String> autoChooser = new SendableChooser<String>();


  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  public final static Axis tAxis = XboxController.Axis.kLeftY;

  /* Subsystems */
  private final SwerveSubsystem swerve = new SwerveSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final ShoulderSubsystem shoulder = new ShoulderSubsystem();
  private final WristSubsystem wrist = new WristSubsystem();
  private final ClawSubsystem claw = new ClawSubsystem();
  private final Limelight vision = new Limelight();


  /* Event Maps (Autos) */
  public static Map<String, Command> linkEventMap1 = new HashMap<>();
  public static Map<String, Command> testeventmap = new HashMap<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. 
   * @throws IOException*/
  public RobotContainer() throws IOException {
    boolean fieldRelative = true;
    boolean openLoop = true;
    swerve.setDefaultCommand(new TeleopSwerve(swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));

    //Event Map for "Link" Autos
    linkEventMap1.put("harvest1", new ArmLowCube(elevator, shoulder, wrist));
    linkEventMap1.put("clawin", new ClawInCommand(claw).withTimeout(0.75));
    linkEventMap1.put("arm1", new ArmMiddle(elevator, shoulder, wrist));
    linkEventMap1.put("clawout", new ClawOutCommand(claw).withTimeout(0.75));
    linkEventMap1.put("return", new ArmLowCone(elevator, shoulder, wrist));

    autoChooser.addOption("charge station test", "charge station test");
    autoChooser.addOption("Link Barrier Side Blue", "Link Barrier Side Blue");
    autoChooser.addOption("Link Barrier Side Red", "Link Barrier Side Red");
    autoChooser.addOption("Link Wall Side Blue", "Link Wall Side Blue");
    autoChooser.addOption("Link Wall Side Red", "Link Wall Side Red");

    SmartDashboard.putData("Auto Selector", autoChooser);

    configureBindings();
  }

  private void configureBindings() throws IOException {
    //Swerve Commands
    new JoystickButton(driver, XboxController.Button.kX.value).onTrue(new InstantCommand(() -> swerve.zeroGyro()));
    new JoystickButton(driver, XboxController.Button.kA.value).whileTrue(new BalanceBrakeCommand(swerve));
    new JoystickButton(driver, XboxController.Button.kB.value).whileTrue(new ClawInCommand(claw));
    new JoystickButton(driver, XboxController.Button.kY.value).whileTrue(new ClawOutCommand(claw));
    new JoystickButton(driver, XboxController.Button.kLeftBumper.value).whileTrue(new AutoBalanceCloseCommand(swerve));
    new JoystickButton(driver, XboxController.Button.kRightBumper.value).whileTrue(new AutoBalanceFarCommand(swerve));
    //new JoystickButton(driver, XboxController.Button.kLeftBumper.value).whileTrue(new TestWristUpCommand(wrist));
    //new JoystickButton(driver, XboxController.Button.kRightBumper.value).whileTrue(new TestWristDownCommand(wrist));
    new JoystickButton(driver, XboxController.Button.kStart.value).whileTrue(new TestShoulderUpCommand(shoulder));
    new JoystickButton(driver, XboxController.Button.kBack.value).whileTrue(new TestShoulderDownCommand(shoulder));
    
    /* */
    new JoystickButton(operator, XboxController.Button.kY.value).whileTrue(new ElevatorTestUpCommand(elevator));
    new JoystickButton(operator, XboxController.Button.kA.value).whileTrue(new ElevatorTestDownCommand(elevator));
    new JoystickButton(operator, XboxController.Button.kStart.value).whileTrue(new TestWristUpCommand(wrist));
    new JoystickButton(operator, XboxController.Button.kBack.value).whileTrue(new TestWristDownCommand(wrist));
    new JoystickButton(operator, XboxController.Button.kRightBumper.value).whileTrue(new TestShoulderUpCommand(shoulder));
    new JoystickButton(operator, XboxController.Button.kLeftBumper.value).whileTrue(new TestShoulderDownCommand(shoulder));
    new JoystickButton(operator, XboxController.Button.kRightStick.value).whileTrue(new ElevatorGoToPosition(elevator, 37));
    new JoystickButton(operator, XboxController.Button.kLeftStick.value).onTrue(new ResetElevatorCommand(elevator));
    new JoystickButton(operator, XboxController.Button.kB.value).whileTrue(new ClawInCommand(claw));
    new JoystickButton(operator, XboxController.Button.kX.value).whileTrue(new ClawOutCommand(claw));
    /*new JoystickButton(operator, XboxController.Button.kLeftStick.value).whileTrue(new GotoPosShoulder(shoulder, 0.3));
    new JoystickButton(operator, XboxController.Button.kB.value).whileTrue(new GoToPositionWrist(wrist, 0.3));
    new JoystickButton(operator, XboxController.Button.kX.value).whileTrue(new ElevatorGoToPosition(elevator, 37));*/
    


    new JoystickButton(buttonBox, 1).onTrue(new ArmTop(elevator, shoulder, wrist));
    new JoystickButton(buttonBox, 2).onTrue(new ArmMiddle(elevator, shoulder, wrist));
    new JoystickButton(buttonBox, 3).onTrue(new ArmLowCone(elevator, shoulder, wrist));
    new JoystickButton(buttonBox, 4).onTrue(new ArmSingle(elevator, shoulder, wrist));
    new JoystickButton(buttonBox, 5).onTrue(new ArmMiddle(elevator, shoulder, wrist));
    new JoystickButton(buttonBox, 6).onTrue(new ArmLowCube(elevator, shoulder, wrist));
    new JoystickButton(buttonBox, 7).onTrue(new ArmRetrieveCone(elevator, shoulder, wrist));
    new JoystickButton(buttonBox, 8).onTrue(new ArmRetrieveCube(elevator, shoulder, wrist));
    new JoystickButton(buttonBox, 9).onTrue(new ArmReturn(elevator, shoulder, wrist));
    new JoystickButton(buttonBox, 10).whileTrue(new HalfSpeed(swerve, driver, translationAxis, strafeAxis, rotationAxis, true, true));
    //new JoystickButton(buttonBox, 10).whileFalse(new InstantCommand(() -> swerve.setDefaultCommand(new TeleopSwerve(swerve, driver, translationAxis, strafeAxis, rotationAxis, true, true))));
    //new JoystickButton(buttonBox, 10).onTrue(new ArmSingle(elevator, shoulder, wrist));
  }

  public void setRumble(){
    driver.setRumble(RumbleType.kLeftRumble, 1);
    driver.setRumble(RumbleType.kRightRumble, 1);
  }

  public void offRumble(){
    driver.setRumble(RumbleType.kLeftRumble, 0);
    driver.setRumble(RumbleType.kRightRumble, 0);
  }


  public Command getAutonomousCommand() {
    String selectedpath = "charge station test";
    PathPlannerTrajectory traj1 = PathPlanner.loadPath(selectedpath, new PathConstraints(2, 1));    
    FollowPathWithEvents command1 = new FollowPathWithEvents(new TestAuto(swerve), traj1.getMarkers(), testeventmap);

    String autobalance = "please";
    PathPlannerTrajectory traj2 = PathPlanner.loadPath(autobalance, new PathConstraints(3, 1.5));
    FollowPathWithEvents command2 = new FollowPathWithEvents(new OverDaStationpt2(swerve), traj2.getMarkers(), testeventmap);
    return new SequentialCommandGroup(/*new ClawOutCommand(claw).withTimeout(0.2),*/ command1, /*new PID_Distance(vision, swerve, driver, translationAxis, strafeAxis, rotationAxis, false, true, 1, 0, -3.9,  2), command2,*/ new AutoAutoBalanceCommand(swerve));
  }
} 