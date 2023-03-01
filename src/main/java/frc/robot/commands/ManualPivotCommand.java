// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;

public class ManualPivotCommand extends CommandBase {
  /** Creates a new ManualPivotCommand. */
  private Pivot pivot;
  private Joystick joystick;
  private boolean isFinished;

  public ManualPivotCommand(Joystick j, Pivot p) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(p);
    pivot = p;
    joystick = j;
    isFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pivotSpeed = joystick.getY();
    this.pivot.PivotDrive(pivotSpeed);
    SmartDashboard.putNumber("Manual Pivot", pivotSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.isFinished = false;
    this.pivot.PivotDrive(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isFinished;
  }
}
