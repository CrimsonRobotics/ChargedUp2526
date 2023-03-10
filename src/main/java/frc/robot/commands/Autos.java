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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public final class Autos {
  /** Example static factory for an autonomous command. */
  
  public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
    return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  }

  public static CommandBase driveOutAuto(Drivetrain d) {
    return Commands.sequence(
      new DriveStraight(d, -0.5),
      new WaitCommand(0.7),
      new DriveStraight(d, 0.5),
      new WaitCommand(3.8),
      new DriveStraight(d, 0)
      // new ManualDrive(d, -0.5, -0.5),
      // new WaitCommand(0.7),
      // new ManualDrive(d, 0.5, 0.5),
      // new WaitCommand(3.8),
      // new ManualDrive(d, 0, 0)
    );
  }

  public static CommandBase ChargeStationAuto(Drivetrain d) {
    return Commands.sequence(
      // new DriveStraight(d, -0.1),
      // new WaitCommand(0.2),
      // new DriveStraight(d, 0.5),
      // new WaitCommand(1.2),
      // new DriveStraight(d, 0),
      // new Balance(d),
      // new DriveStraight(d, 0)
      new ManualDrive(d, -0.2, -0.2),
      new WaitCommand(0.5),
      new ManualDrive(d, 0.5, 0.5),
      new WaitCommand(1.5),
      new ManualDrive(d, 0, 0),
      new Balance(d),
      new ManualDrive(d,0, 0)//,
      //new WaitUntilCommand(() -> !robot.isauto()))
    );
  }

  public static CommandBase oneScoreLeftAuto(Joystick j, Pivot p, Wrist w, Telescope t, Claw c, Drivetrain d){
    return Commands.sequence(
      // new ParallelCommandGroup(
      //   new PivotHoldCommand(j, p, Constants.outtakeHigh, false),
      //   new WristDrive(w, Constants.outtakeHigh, true),
      //   new TelescopeDrive(t, Constants.outtakeHigh),
      //   new SequentialCommandGroup(
      //     new WaitCommand(3),
      //     new OuttakeCommand(c, 0.5),
      //     new WaitCommand(2),
      //     new OuttakeCommand(c, 0),
      //     new ParallelCommandGroup(
      //       new PivotHoldCommand(j, p, Constants.travel, false),
      //       new WristDrive(w, Constants.travel, true),
      //       new TelescopeDrive(t, Constants.travel)
      //     ),
      //     new DriveStraight(d, 0.5),
      //     new WaitCommand(5),
      //     new DriveStraight(d, 0)
      //   )
      // )
    );

  }

  public static CommandBase oneScoreEngageAuto(Joystick j, Pivot p, Wrist w, Telescope t, Claw c, Drivetrain d){
    return Commands.sequence(
      // new ParallelCommandGroup(
      //   new PivotHoldCommand(j, p, Constants.outtakeHigh, false),
      //   new WristDrive(w, Constants.outtakeHigh, true),
      //   new TelescopeDrive(t, Constants.outtakeHigh),
      //   new SequentialCommandGroup(
      //     new WaitCommand(3),
      //     new OuttakeCommand(c, 0.5),
      //     new WaitCommand(2),
      //     new OuttakeCommand(c, 0),
      //     new ParallelCommandGroup(
      //       new PivotHoldCommand(j, p, Constants.travel, true),
      //       new WristDrive(w, Constants.travel, true),
      //       new TelescopeDrive(t, Constants.travel)
      //     ),
      //     new DriveStraight(d, 0.5),
      //     new WaitCommand(5),
      //     new DriveStraight(d, 0),
      //     new Balance(d)
      //   )
      // ) 
      
    );
  }

  public static CommandBase oneScoreRightAuto(Drivetrain d, Joystick j, Pivot p, Wrist w, Telescope t){
    return Commands.sequence(
      new SequentialCommandGroup(
        // new PivotHoldCommand(j, p, Constants.outtakeHigh, true, 5),
        // new PivotHoldCommand(j, p, Constants.travel, false, 0)
        // new PivotHoldCommand(j, p, Constants.travel, false).alongWith(new WristDrive(w, Constants.travel, false))

        // new DriveStraight(d, 0),
        // new WaitCommand(3),
        // new ParallelCommandGroup(
        //   new PivotHoldCommand(j, p, Constants.outtakeHigh),
        //   new WristDrive(w, Constants.outtakeHigh),
        //   new TelescopeDrive(t, Constants.outtakeHigh),
        //   new SequentialCommandGroup(
        //     new WaitCommand(3),
        //     new ArmCancelCommand(p, t, w)
        //     // new ParallelCommandGroup(
        //     //   new PivotHoldCommand(j, p, Constants.intakeLow),
        //     //   new WristDrive(w, Constants.intakeLow),
        //     //   new TelescopeDrive(t, Constants.intakeLow)
        //     // )
        //   )

        // )
      )
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
