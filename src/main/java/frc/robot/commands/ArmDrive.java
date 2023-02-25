// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.Arm;

public class ArmDrive extends CommandBase {
  /** Creates a new ArmDrive. */
  double armState[];
  double originalPivot;
  double extensionPos;
  boolean isFinished;
  public ArmDrive(double ac[]) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(Robot.arm);
    armState = ac;
    isFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    originalPivot = Robot.arm.pivotPot.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    extensionPos = Robot.arm.extendPot.get();
    if(Robot.arm.armState == true){
      SmartDashboard.putString("Game Piece", "Cone");
      if(Math.abs(originalPivot-armState[0])<30){
        SmartDashboard.putString("extending", "yes");
        new ParallelCommandGroup(
          new PivotDrive(armState),
          new TelescopeDrive(armState),
          new WristDrive(armState)
        );
        // new PivotDrive(armState);
        // new TelescopeDrive(armState);
        // new WristDrive(armState);
      }
      else{
        SmartDashboard.putString("extending", "no");

        new SequentialCommandGroup(
          new ParallelCommandGroup(
            new PivotDrive(armState),
            new WristDrive(armState)
          ),
          new TelescopeDrive(armState)
        );
      }
    }
    else{
      SmartDashboard.putString("Game Piece", "Cube");

      if(Math.abs(originalPivot-armState[3])<30){
        SmartDashboard.putString("arm moving", "yes");
        new ParallelCommandGroup(
          new PivotDrive(armState),
          new TelescopeDrive(armState),
          new WristDrive(armState)
        );
      }
      else{
        SmartDashboard.putString("arm moving", "no");

        new SequentialCommandGroup(
          new ParallelCommandGroup(
            new PivotDrive(armState),
            new WristDrive(armState)
          ),
          new TelescopeDrive(armState)
        );
      }
    }
    if(Math.abs(extensionPos-armState[1])<2){
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
