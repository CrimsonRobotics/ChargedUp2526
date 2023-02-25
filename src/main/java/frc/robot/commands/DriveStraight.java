// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.PIDConstants;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain;

public class DriveStraight extends CommandBase {
  private Drivetrain driveTrain;
  double driveSpeed;
  double originalYaw;
  /** Creates a new DriveStraight. */
  public DriveStraight(Drivetrain d, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    // double rightSpeed = rS;
    driveTrain = d;
    addRequirements(this.driveTrain);
    driveSpeed = speed;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    originalYaw = this.driveTrain.pigeon.getYaw() % 360;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pigeonReadout = this.driveTrain.pigeon.getYaw() % 360;
    SmartDashboard.putNumber("Gyro Heading", pigeonReadout);
    SmartDashboard.putNumber("original Heading", originalYaw);

    double speed = MathUtil.clamp(this.driveTrain.straightPID.calculate(pigeonReadout, originalYaw), -PIDConstants.straightMaxPercent, PIDConstants.straightMaxPercent);
    speed = speed / 100;
    // this.driveTrain.TeleopDrive(0, -speed);
    this.driveTrain.TeleopDrive(driveSpeed, -speed);

    // this.driveTrain.ManualDrive(-speed, speed);

    // SmartDashboard.putNumber("pigeonAlignSpeed", speed);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
