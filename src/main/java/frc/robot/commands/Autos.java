// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  
  public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static CommandBase oneScoreLeftAuto(Joystick j, Pivot p, Wrist w, Telescope t, Claw c, Drivetrain d){
    return Commands.sequence(
      new ParallelCommandGroup(
        new PivotHoldCommand(j, p, Constants.outtakeHigh),
        new WristDrive(w, Constants.outtakeHigh),
        new TelescopeDrive(t, Constants.outtakeHigh),
        new SequentialCommandGroup(
          new WaitCommand(3),
          new OuttakeCommand(c, 0.5),
          new WaitCommand(2),
          new OuttakeCommand(c, 0),
          new ParallelCommandGroup(
            new PivotHoldCommand(j, p, Constants.travel),
            new WristDrive(w, Constants.travel),
            new TelescopeDrive(t, Constants.travel)
          ),
          new DriveStraight(d, 0.5),
          new WaitCommand(5),
          new DriveStraight(d, 0)
        )
      )
    );

  }

  public static CommandBase oneScoreEngageAuto(Joystick j, Pivot p, Wrist w, Telescope t, Claw c, Drivetrain d){
    return Commands.sequence(
      new ParallelCommandGroup(
        new PivotHoldCommand(j, p, Constants.outtakeHigh),
        new WristDrive(w, Constants.outtakeHigh),
        new TelescopeDrive(t, Constants.outtakeHigh),
        new SequentialCommandGroup(
          new WaitCommand(3),
          new OuttakeCommand(c, 0.5),
          new WaitCommand(2),
          new OuttakeCommand(c, 0),
          new ParallelCommandGroup(
            new PivotHoldCommand(j, p, Constants.travel),
            new WristDrive(w, Constants.travel),
            new TelescopeDrive(t, Constants.travel)
          ),
          new DriveStraight(d, 0.5),
          new WaitCommand(5),
          new DriveStraight(d, 0),
          new Balance(d)
        )
      ) 
      
    );
  }

  public static CommandBase oneScoreRightAuto(){
    return Commands.sequence(
      
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
