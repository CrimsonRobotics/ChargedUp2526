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
import frc.robot.subsystems.Pivot;

public class PivotDrive extends CommandBase {
  /** Creates a new PivotDrive. */
  private Pivot pivot;
  double armcase[];
  boolean isFinished;
  public PivotDrive(Pivot p, double ac[]) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(p);
    pivot = p;
    armcase = ac;
    isFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.printf("Pivot Drive Command Executing");

    double pivotPotReadout = this.pivot.pivotPot.get();

    if(Pivot.armState == true){

      double pivotspeed = MathUtil.clamp(this.pivot.pivotPID.calculate(pivotPotReadout, armcase[0]), -PIDConstants.pivotMaxPercent, PIDConstants.pivotMaxPercent);
      pivotspeed = pivotspeed / 100;
      this.pivot.PivotDrive(pivotspeed);
      SmartDashboard.putNumber("Pivot Speed",pivotspeed);
    }
    else if(Pivot.armState == false){

      double pivotspeed = MathUtil.clamp(this.pivot.pivotPID.calculate(pivotPotReadout, armcase[3]), -PIDConstants.pivotMaxPercent, PIDConstants.pivotMaxPercent);
      pivotspeed = pivotspeed / 100;
      this.pivot.PivotDrive(pivotspeed);
      SmartDashboard.putNumber("Pivot Speed",pivotspeed);
    }
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
