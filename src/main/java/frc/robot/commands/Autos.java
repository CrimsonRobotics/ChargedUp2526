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

  public static CommandBase overChargeAuto(Joystick j, Pivot p, Wrist w, Telescope t, Claw c, Drivetrain d){
    return Commands.sequence(
      //Score High
        new PivotHoldCommand(j, p, Constants.outtakeHigh).alongWith(new TelescopeDrive(j, t, Constants.outtakeHigh)).alongWith(new WristDrive(j, w, Constants.outtakeHigh, false)).alongWith(new IntakeHoldCommand(c, 0.6)).raceWith(new WaitCommand(3)),
        new OuttakeCommand(c, 1).raceWith(new WaitCommand(0.5)),
        new OuttakeCommand(c, 0).raceWith(new WaitCommand(0.1)),
        //Drive Forward over Charge Station
        new PivotHoldCommand(j, p, Constants.travel).alongWith(new TelescopeDrive(j, t, Constants.travel)).alongWith(new WristDrive(j, w, Constants.travel, false)).raceWith(new WaitCommand(1)),
        new PivotHoldCommand(j, p, Constants.travel).alongWith(new TelescopeDrive(j, t, Constants.travel)).alongWith(new WristDrive(j, w, Constants.travel, false))
        .alongWith(new ManualDrive(d, 0.4, 0.4)).raceWith(new WaitCommand(1.2)),
        new PivotHoldCommand(j, p, Constants.travel).alongWith(new TelescopeDrive(j, t, Constants.travel)).alongWith(new WristDrive(j, w, Constants.travel, false))
        .alongWith(new ManualDrive(d,0.2, 0.2)).raceWith(new WaitCommand(1.2)),
        new PivotHoldCommand(j, p, Constants.travel).alongWith(new TelescopeDrive(j, t, Constants.travel)).alongWith(new WristDrive(j, w, Constants.travel, false))
        .alongWith(new ManualDrive(d,0, 0)).raceWith(new WaitCommand(1)),
        //Drive back onto charge station
        // new PivotHoldCommand(j, p, Constants.travel).alongWith(new TelescopeDrive(j, t, Constants.travel)).alongWith(new WristDrive(j, w, Constants.travel, false))
        // .alongWith(new ManualDrive(d,-0.2, -0.2)).raceWith(new WaitCommand(0.5)),
        // new PivotHoldCommand(j, p, Constants.travel).alongWith(new TelescopeDrive(j, t, Constants.travel)).alongWith(new WristDrive(j, w, Constants.travel, false))
        // .alongWith(new ManualDrive(d,0, 0)).raceWith(new WaitCommand(1)),
        // //Balance
        // new PivotHoldCommand(j, p, Constants.travel).alongWith(new TelescopeDrive(j, t, Constants.travel)).alongWith(new WristDrive(j, w, Constants.travel, false)).alongWith(new Balance(d)),
        new PivotHoldCommand(j, p, Constants.travel).alongWith(new TelescopeDrive(j, t, Constants.travel)).alongWith(new WristDrive(j, w, Constants.travel, false))
      
    );
  }

  public static CommandBase oneScoreCloseAuto(Joystick j, Pivot p, Wrist w, Telescope t, Claw c, Drivetrain d){
    return Commands.sequence(
      //Score one piece
      new PivotHoldCommand(j, p, Constants.outtakeHigh).alongWith(new TelescopeDrive(j, t, Constants.outtakeHigh)).alongWith(new WristDrive(j, w, Constants.outtakeHigh, false)).alongWith(new IntakeHoldCommand(c, 0.6)).raceWith(new WaitCommand(3)),
      new OuttakeCommand(c, 1).raceWith(new WaitCommand(0.5)),
      new OuttakeCommand(c, 0).raceWith(new WaitCommand(0.1)),
      new PivotHoldCommand(j, p, Constants.travel).alongWith(new TelescopeDrive(j, t, Constants.travel)).alongWith(new WristDrive(j, w, Constants.travel, false)).raceWith(new WaitCommand(1)),
     //Drie out short community
      new PivotHoldCommand(j, p, Constants.travel).alongWith(new TelescopeDrive(j, t, Constants.travel)).alongWith(new WristDrive(j, w, Constants.travel, false))
      .alongWith(new ManualDrive(d, 0.2, 0.2)).raceWith(new WaitCommand(3.5)),
      new PivotHoldCommand(j, p, Constants.travel).alongWith(new TelescopeDrive(j, t, Constants.travel)).alongWith(new WristDrive(j, w, Constants.travel, false))
      .alongWith(new ManualDrive(d, 0, 0)).raceWith(new WaitCommand(1)),
      new PivotHoldCommand(j, p, Constants.travel).alongWith(new TelescopeDrive(j, t, Constants.travel)).alongWith(new WristDrive(j, w, Constants.travel, false))
      
    );

  }

  public static CommandBase oneScoreEngageAuto(Joystick j, Pivot p, Wrist w, Telescope t, Claw c, Drivetrain d){
    return Commands.sequence(
      //Score high
        new PivotHoldCommand(j, p, Constants.outtakeHigh).alongWith(new TelescopeDrive(j, t, Constants.outtakeHigh)).alongWith(new WristDrive(j, w, Constants.outtakeHigh, false)).alongWith(new IntakeHoldCommand(c, 0.6)).raceWith(new WaitCommand(2.6)),
        new OuttakeCommand(c, 1).raceWith(new WaitCommand(0.5)),
        new OuttakeCommand(c, 0).raceWith(new WaitCommand(0.1)),
        new PivotHoldCommand(j, p, Constants.travel).alongWith(new TelescopeDrive(j, t, Constants.travel)).alongWith(new WristDrive(j, w, Constants.travel, false)).raceWith(new WaitCommand(1)),
       //Drive to charge station
        new PivotHoldCommand(j, p, Constants.travel).alongWith(new TelescopeDrive(j, t, Constants.travel)).alongWith(new WristDrive(j, w, Constants.travel, false))
        .alongWith(new ManualDrive(d, 0.4, 0.4)).raceWith(new WaitCommand(1.25)),
        new PivotHoldCommand(j, p, Constants.travel).alongWith(new TelescopeDrive(j, t, Constants.travel)).alongWith(new WristDrive(j, w, Constants.travel, false))
        .alongWith(new ManualDrive(d, 0, 0)).raceWith(new WaitCommand(0.1)),
        //Balance
        new PivotHoldCommand(j, p, Constants.travel).alongWith(new TelescopeDrive(j, t, Constants.travel)).alongWith(new WristDrive(j, w, Constants.travel, false)).alongWith(new Balance(d)),
        new PivotHoldCommand(j, p, Constants.travel).alongWith(new TelescopeDrive(j, t, Constants.travel)).alongWith(new WristDrive(j, w, Constants.travel, false))

        );
     
      
    
  }

  public static CommandBase oneScoreFarAuto(Drivetrain d, Joystick j, Pivot p, Wrist w, Telescope t, Claw c){
    return Commands.sequence(
      //Score high
      new PivotHoldCommand(j, p, Constants.outtakeHigh).alongWith(new TelescopeDrive(j, t, Constants.outtakeHigh)).alongWith(new WristDrive(j, w, Constants.outtakeHigh, false)).alongWith(new IntakeHoldCommand(c, 0.6)).raceWith(new WaitCommand(3)),
      new OuttakeCommand(c, 1).raceWith(new WaitCommand(0.5)),
      new OuttakeCommand(c, 0).raceWith(new WaitCommand(0.1)),
      new PivotHoldCommand(j, p, Constants.travel).alongWith(new TelescopeDrive(j, t, Constants.travel)).alongWith(new WristDrive(j, w, Constants.travel, false)).raceWith(new WaitCommand(1)),
      //Drive out of long community
      new PivotHoldCommand(j, p, Constants.travel).alongWith(new TelescopeDrive(j, t, Constants.travel)).alongWith(new WristDrive(j, w, Constants.travel, false))
      .alongWith(new ManualDrive(d, 0.2, 0.2)).raceWith(new WaitCommand(4.5)),
      new PivotHoldCommand(j, p, Constants.travel).alongWith(new TelescopeDrive(j, t, Constants.travel)).alongWith(new WristDrive(j, w, Constants.travel, false))
      .alongWith(new ManualDrive(d, 0, 0)).raceWith(new WaitCommand(1)),
      new PivotHoldCommand(j, p, Constants.travel).alongWith(new TelescopeDrive(j, t, Constants.travel)).alongWith(new WristDrive(j, w, Constants.travel, false))
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
