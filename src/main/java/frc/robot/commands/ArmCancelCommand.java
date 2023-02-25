// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmCancelCommand extends InstantCommand {
//Cancels all arm movement
private Pivot pivot;
private Telescope telescope;
private Wrist wrist;

public ArmCancelCommand(Pivot p, Telescope t, Wrist w) {
    // Use addRequirements() here to declare subsystem dependencies.
  addRequirements(p, t, w);
  pivot = p;
  telescope = t;
  wrist = w;  
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}
}
