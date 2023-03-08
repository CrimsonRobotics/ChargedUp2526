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
import frc.robot.subsystems.Drivetrain;

public class Balance extends CommandBase {
  private Drivetrain driveTrain;
  /** Creates a new Balance. */
  boolean isFinished;
  public Balance(Drivetrain dt) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dt);
    driveTrain = dt;
    this.isFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rollReadout = this.driveTrain.pigeon.getPitch() % 360;

    double speed = MathUtil.clamp(this.driveTrain.balancePID.calculate(rollReadout, PIDConstants.balanceSetpoint), -PIDConstants.balanceMaxPercent, PIDConstants.balanceMaxPercent);
    speed = speed / 100;
    this.driveTrain.TeleopDrive(-speed, 0);
    
    if (Math.abs(rollReadout-PIDConstants.balanceSetpoint)<Constants.balanceError){
      this.isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.driveTrain.ManualDrive(0, 0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.isFinished;
  }
}
