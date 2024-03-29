// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Pivot;

public class IntakeCommand extends CommandBase {
  /** Creates a new IntakeCommand. */
  private Claw claw;
  private double speed;
  private boolean isFinished;
  public IntakeCommand(Claw c, double s) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(c);
    claw = c;
    speed = s;
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
    if (Pivot.armState == true){
      this.claw.intakeMotor.setSmartCurrentLimit(25);
    }
    else if (Pivot.armState == false){
      this.claw.intakeMotor.setSmartCurrentLimit(25);
    }
    this.claw.IntakeDrive(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.isFinished = true;
    this.claw.IntakeDrive(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isFinished;
  }
}
