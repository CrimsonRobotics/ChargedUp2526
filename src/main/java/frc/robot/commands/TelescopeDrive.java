// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.PIDConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Telescope;

public class TelescopeDrive extends CommandBase {
  /** Creates a new TelescopeDrive. */
  private Telescope telescope;
  boolean isFinished;
  double armcase[];
  Joystick joystick;
  public TelescopeDrive(Joystick j, Telescope t, double ac[]) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(t);
    telescope = t;
    isFinished = false;
    armcase = ac;
    joystick = j;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.printf("Telescope Drive Command Executing");

    double telescopePotReadout = this.telescope.telescopePot.get();
    double pivotPotReadout = Pivot.pivotPot.get();
    // double telescopePotReadout = 15;
    double adjust = 0;
    if(joystick.getPOV() == 0){
      adjust = 10;
    }
    else if(joystick.getPOV() == 180){
      adjust = -10;
    }
    else{
      adjust = 0;
    }
    
    if(Pivot.armState == true){
        if((armcase[2]-telescopePotReadout)<0){
          double telescopespeed = MathUtil.clamp(this.telescope.downPID.calculate(telescopePotReadout, armcase[2]), -PIDConstants.downMaxPercent, PIDConstants.downMaxPercent);
          telescopespeed = telescopespeed / 100;
          this.telescope.telescopeDrive(telescopespeed);
          // SmartDashboard.putNumber("telescope Speed",telescopespeed);
          // SmartDashboard.putNumber("telescope Speed", telescopespeed);
          // SmartDashboard.putString("waiting for pivot", "No");
        }
        else {
          double telescopespeed = MathUtil.clamp(this.telescope.telescopePID.calculate(telescopePotReadout, armcase[2]+adjust), -PIDConstants.telescopeMaxPercent, PIDConstants.telescopeMaxPercent);
          telescopespeed = telescopespeed / 100;
          this.telescope.telescopeDrive(telescopespeed);
          // SmartDashboard.putNumber("telescope Speed",telescopespeed);
          // SmartDashboard.putNumber("telescope Speed", telescopespeed);
          // SmartDashboard.putString("waiting for pivot", "No");
        }
        
      
    }
    else if(Pivot.armState == false){
        if((armcase[5]-telescopePotReadout)<0){
          double telescopespeed = MathUtil.clamp(this.telescope.downPID.calculate(telescopePotReadout, armcase[5]), -PIDConstants.downMaxPercent, PIDConstants.downMaxPercent);
          telescopespeed = telescopespeed / 100;
          this.telescope.telescopeDrive(telescopespeed);
          // SmartDashboard.putNumber("telescope Speed",telescopespeed);
          // SmartDashboard.putNumber("telescope Speed", telescopespeed);
          // SmartDashboard.putString("waiting for pivot", "No");
        }
        else {
          double telescopespeed = MathUtil.clamp(this.telescope.telescopePID.calculate(telescopePotReadout, armcase[5]+adjust), -PIDConstants.telescopeMaxPercent, PIDConstants.telescopeMaxPercent);
          telescopespeed = telescopespeed / 100;
          this.telescope.telescopeDrive(telescopespeed);
          // SmartDashboard.putNumber("telescope Speed",telescopespeed);
          // SmartDashboard.putNumber("telescope Speed", telescopespeed);
          // SmartDashboard.putString("waiting for pivot", "No");
        }
      }
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.isFinished = true;
    // this.telescope.telescopeDrive(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isFinished;
  }
}
