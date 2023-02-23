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

public class AbsoluteTurn extends CommandBase {
  /** Creates a new Align. */
  double turnPoint;
  boolean isFinished;
  public AbsoluteTurn(double tp) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driveTrain);
    turnPoint = tp;
    isFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double yawReadout = Robot.driveTrain.pigeon.getYaw() % 360;
    double speed = MathUtil.clamp(Robot.driveTrain.turnPID.calculate(yawReadout, turnPoint), -PIDConstants.pidMaxPercent, PIDConstants.pidMaxPercent);
    speed = speed / 100;
    Robot.driveTrain.TeleopDrive(0, -speed);

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
