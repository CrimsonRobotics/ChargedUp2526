// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveStraight extends CommandBase {
  double driveSpeed;
  double originalYaw;
  /** Creates a new DriveStraight. */
  public DriveStraight(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
     driveSpeed = speed;
    // double rightSpeed = rS;
    addRequirements(Robot.driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // originalYaw = Robot.driveTrain.pigeon.getYaw() % 360;
    originalYaw = Robot.driveTrain.gyro.getAngle() % 360;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double pigeonReadout = Robot.driveTrain.pigeon.getYaw() % 360;
    // SmartDashboard.putNumber("Gyro Heading", pigeonReadout);
    // SmartDashboard.putNumber("original Heading", originalYaw);

    // double speed = MathUtil.clamp(Robot.driveTrain.turnPID.calculate(pigeonReadout, originalYaw), -Constants.pidMaxPercent, Constants.pidMaxPercent);
    // speed = speed / 100;
    // // Robot.driveTrain.TeleopDrive(0, -speed);
    // Robot.driveTrain.TeleopDrive(driveSpeed, -speed);

    // // Robot.driveTrain.ManualDrive(-speed, speed);

    // SmartDashboard.putNumber("pigeonAlignSpeed", speed);
    double gyroReadout = Robot.driveTrain.gyro.getAngle() % 360;
    SmartDashboard.putNumber("Gyro Heading", gyroReadout);
    SmartDashboard.putNumber("original Heading", originalYaw);

    double speed = MathUtil.clamp(Robot.driveTrain.turnPID.calculate(gyroReadout, originalYaw), -Constants.pidMaxPercent, Constants.pidMaxPercent);
    speed = speed / 100;
    // Robot.driveTrain.TeleopDrive(0, -speed);
    Robot.driveTrain.TeleopDrive(driveSpeed, -speed);

    // Robot.driveTrain.ManualDrive(-speed, speed);

    SmartDashboard.putNumber("pigeonAlignSpeed", speed);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
