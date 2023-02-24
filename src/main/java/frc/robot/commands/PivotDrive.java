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

public class PivotDrive extends CommandBase {
  /** Creates a new PivotDrive. */
  double armcase[];
  boolean isFinished;
  public PivotDrive(double ac[]) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.arm);
    armcase = ac;
    isFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pivotPotReadout = Robot.arm.pivotPot.get();

    double pivotspeed = MathUtil.clamp(Robot.arm.pivotPID.calculate(pivotPotReadout, armcase[0]), -PIDConstants.pivotMaxPercent, PIDConstants.pivotMaxPercent);
    pivotspeed = pivotspeed / 100;
    Robot.arm.PivotDrive(pivotspeed);
    // SmartDashboard.putNumber("Arm Speed",pivotspeed);
    if (Math.abs(pivotPotReadout-armcase[0])<Constants.pivotError){
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
