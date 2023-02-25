// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.Claw;

public class ToggleIntake extends CommandBase {
  /** Creates a new ToggleIntake. */
  private Claw claw;
  boolean intakeState;
  public ToggleIntake(Claw c, boolean state) {
    // Use addRequirements() here to declare subsystem dependencies.
    claw = c;
    addRequirements(c);
    intakeState = state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intakeState == true){
      this.claw.intakeSolenoid.set(Value.kForward);
    }
    else {
      this.claw.intakeSolenoid.set(Value.kReverse);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
