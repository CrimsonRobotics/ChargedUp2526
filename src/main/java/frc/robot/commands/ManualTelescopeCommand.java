// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.Telescope;

public class ManualTelescopeCommand extends CommandBase {
  /** Creates a new ManualTelescopeCommand. */
  private Telescope telescope;

  private Joystick joystick;
  private boolean isFinished;

  public ManualTelescopeCommand(Joystick j, Telescope t) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(t);
    telescope = t;
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
    if(joystick.getPOV() == 0){
      this.telescope.telescopeDrive(0.6);
      // SmartDashboard.putString("Extending", "Up");

    }
    else if(joystick.getPOV() == 180){
      this.telescope.telescopeDrive(-0.2);
      // SmartDashboard.putString("Extending", "Down");
    }
    else{
      this.telescope.telescopeDrive(0);
      // SmartDashboard.putString("Extending", "No");

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    this.isFinished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isFinished;
  }
}
