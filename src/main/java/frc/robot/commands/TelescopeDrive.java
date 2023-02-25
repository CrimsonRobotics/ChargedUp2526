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

public class TelescopeDrive extends CommandBase {
  /** Creates a new TelescopeDrive. */
  boolean isFinished;
  double armcase[];
  public TelescopeDrive(double ac[]) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(Robot.arm);
    isFinished = false;
    armcase = ac;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    double extendPotReadout = Robot.arm.extendPot.get();
    // double extendPotReadout = 15;
    if(Robot.arm.armState == true){
      double extendspeed = MathUtil.clamp(Robot.arm.extendPID.calculate(extendPotReadout, armcase[3]), -PIDConstants.extendMaxPercent, PIDConstants.extendMaxPercent);
      extendspeed = extendspeed / 100;
      Robot.arm.ExtendDrive(extendspeed);
      // SmartDashboard.putNumber("Extend Speed",extendspeed);
      SmartDashboard.putNumber("Extend Speed", extendspeed);
        if (Math.abs(extendPotReadout-armcase[3])<Constants.telescopeError){
        isFinished = true;
        }
      }
      else if(Robot.arm.armState == false){
        double extendspeed = MathUtil.clamp(Robot.arm.extendPID.calculate(extendPotReadout, armcase[5]), -PIDConstants.extendMaxPercent, PIDConstants.extendMaxPercent);
        extendspeed = extendspeed / 100;
        Robot.arm.ExtendDrive(extendspeed);
        // SmartDashboard.putNumber("Extend Speed",extendspeed);
        SmartDashboard.putNumber("Extend Speed",extendspeed);
        if (Math.abs(extendPotReadout-armcase[5])<Constants.telescopeError){
        isFinished = true;
        }
      }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
