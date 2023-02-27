// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class ManualWristCommand extends CommandBase {
  /** Creates a new ManualWristCommand. */
  private Wrist wrist;
  private Joystick joystick;
  
  public ManualWristCommand(Joystick j, Wrist w) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(w);
    joystick = j;
    wrist = w;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(joystick.getPOV() == 0){
      this.wrist.WristDrive(0.5);
      SmartDashboard.putString("Wrist", "Up");

    }
    else if(joystick.getPOV() == 180){
      this.wrist.WristDrive(-0.3);
      SmartDashboard.putString("Wrist", "Down");
    }
    else{
      this.wrist.WristDrive(0);
      SmartDashboard.putString("Wrist", "No");

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
