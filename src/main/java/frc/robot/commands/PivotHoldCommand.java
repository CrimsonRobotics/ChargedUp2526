// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.sound.sampled.Line;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.PIDConstants;
import frc.robot.Robot;
import frc.robot.subsystems.Pivot;


public class PivotHoldCommand extends CommandBase {
  /** Creates a new PivotDrive. */
  private Pivot pivot;
  double armcase[];
  boolean isFinished;
  private Joystick joystick;
  private LinearFilter filter;
  // Timer timer;
  // Boolean timed;
  // double time;
  public PivotHoldCommand(Joystick j, Pivot p, double ac[]/*, boolean t, double tr*/) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(p);
    joystick = j;
    pivot = p;
    armcase = ac;
    isFinished = false;
    // timed = t;
    // time = tr;
    filter = LinearFilter.singlePoleIIR(0.1, 0.02);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // timer.start();
    this.isFinished = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.printf("Pivot Drive Command Executing");
    double adjust = joystick.getY()*5;

    double pivotPotReadout = this.pivot.pivotPot.get();
    

    if(Pivot.armState == true){
      

      double pivotspeed = MathUtil.clamp(this.pivot.pivotPID.calculate(this.filter.calculate(pivotPotReadout), armcase[0]+adjust), -PIDConstants.pivotMaxPercent, PIDConstants.pivotMaxPercent);
      pivotspeed = pivotspeed / 100;
      this.pivot.PivotDrive(-pivotspeed);
      // SmartDashboard.putNumber("Pivot Speed",-pivotspeed);
    }
    else if(Pivot.armState == false){

      double pivotspeed = MathUtil.clamp(this.pivot.pivotPID.calculate(this.filter.calculate(pivotPotReadout), armcase[3]+adjust), -PIDConstants.pivotMaxPercent, PIDConstants.pivotMaxPercent);
      pivotspeed = pivotspeed / 100;
      this.pivot.PivotDrive(-pivotspeed);
      // SmartDashboard.putNumber("Pivot Speed",-pivotspeed);
    }
    // if(timed == true){
    //   if(timer.get()<time){
    //     isFinished = true;
    //   }
    //   else{
    //     isFinished = false;
    //   }
    // }
    // else {
    //   isFinished = false;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // this.pivot.PivotDrive(0);
    this.isFinished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isFinished;
  }
}
