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
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Wrist;

public class WristDrive extends CommandBase {
  /** Creates a new WristDrive. */
  boolean isFinished;
  private Wrist wrist;
  double armcase[];
  Joystick joystick;

  public WristDrive(Joystick j, Wrist w, double ac[], boolean f) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(w);
    wrist = w;
    isFinished = f;
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
    System.out.printf("Wrist Drive Command Executing");

    double wristPotReadout = (this.wrist.wristPot.getPosition()/3.29*360);
    double adjust = 0;
    if(joystick.getPOV() == 0){
      adjust = 5;
    }
    else if(joystick.getPOV() == 180){
      adjust = -5;
    }
    else{
      adjust = 0;
    }

    if(Pivot.armState == true){
      double wristspeed = MathUtil.clamp(this.wrist.wristPID.calculate(wristPotReadout, armcase[1]+adjust), -PIDConstants.wristMaxPercent, PIDConstants.wristMaxPercent);
      wristspeed = wristspeed / 100;
      this.wrist.WristDrive(-wristspeed);
      SmartDashboard.putNumber("Wrist Speed",wristspeed);
    }
    else if(Pivot.armState == false){
      double wristspeed = MathUtil.clamp(this.wrist.wristPID.calculate(wristPotReadout, armcase[4]+adjust), -PIDConstants.wristMaxPercent, PIDConstants.wristMaxPercent);
      wristspeed = wristspeed / 100;
      this.wrist.WristDrive(-wristspeed);
      SmartDashboard.putNumber("Wrist Speed",wristspeed);
    }
    // double wristPotReadout = 100;
    // SmartDashboard.putNumber("Wrist Speed",wristspeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.isFinished = true;
    // this.wrist.WristDrive(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isFinished ;
  }
}
