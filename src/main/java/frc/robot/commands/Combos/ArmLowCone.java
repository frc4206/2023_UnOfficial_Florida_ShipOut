// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Combos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.Elevator.ElevatorGoToPosition;
import frc.robot.commands.Arm.Shoulder.GotoPosShoulder;
import frc.robot.commands.Arm.Wrist.GoToPositionWrist;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmLowCone extends SequentialCommandGroup {
  public ArmLowCone(ElevatorSubsystem elevator, ShoulderSubsystem shoulder, WristSubsystem wrist) {
    
    addCommands(
      new ParallelCommandGroup(
        new ElevatorGoToPosition(elevator, 30.40).withTimeout(2),
        new GotoPosShoulder(shoulder, Constants.Arm.Shoulder.targetPosition3).withTimeout(1),
        new GoToPositionWrist(wrist, 0.252).withTimeout(1)
      ).withTimeout(1.5)
    );
  }
}
