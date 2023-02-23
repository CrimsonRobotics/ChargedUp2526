// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.apriltag.jni.AprilTagJNI.Helper;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Helpers;
import frc.robot.PIDConstants;
import frc.robot.Robot;

public class RelativeTurn extends CommandBase {
  /** Creates a new TurnPrecise. */
  double turnPoint;
  boolean isFinished;
  double originalYaw;
  public RelativeTurn(double tp) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
    turnPoint = tp;
    isFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    originalYaw = Robot.driveTrain.pigeon.getYaw() % 360;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yawReadout = Robot.driveTrain.pigeon.getYaw() % 360;
    // SmartDashboard.putNumber("Gyro Heading", gyroReadout);
    // SmartDashboard.putNumber("original Heading", originalYaw);

    double speed = MathUtil.clamp(Robot.driveTrain.turnPID.calculate(yawReadout, originalYaw + turnPoint), -PIDConstants.pidMaxPercent, PIDConstants.pidMaxPercent);
    speed = speed / 100;
    Robot.driveTrain.TeleopDrive(0, -speed);
    // if(Helpers.isTurnCCW(originalYaw, originalYaw+turnPoint)){
    //   double newPoint = Helpers.hdgDiff(originalYaw, turnPoint);
    //   speed = MathUtil.clamp(Robot.driveTrain.turnPID.calculate(gyroReadout, originalYaw + newPoint), -Constants.pidMaxPercent, Constants.pidMaxPercent);
    //   speed = speed / 100;
    //   Robot.driveTrain.TeleopDrive(0, speed);

    // }
    // else {
    //   Robot.driveTrain.TeleopDrive(0, -speed);
    // }

    if (Math.abs(yawReadout-turnPoint)<Constants.alignError){
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
