// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Wrist;

public class ArmHoldCommand extends CommandBase {
  /** Creates a new ArmDrive. */
  double armState[];
  double originalPivot;
  double extensionPos;
  boolean isFinished;
  private Claw claw;
  private Pivot pivot;
  private Telescope telescope;
  private Wrist wrist;
  
  public ArmHoldCommand(Pivot p, Telescope t, Wrist w, double ac[]) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(p, t, w);
    pivot = p;
    telescope = t;
    wrist = w;
    armState = ac;
    isFinished = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    originalPivot = this.pivot.pivotPot.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.printf("Arm Drive Command Executing");
        

    CommandScheduler.getInstance().schedule(new ParallelCommandGroup(
        new PivotDrive(this.pivot, armState),
        new TelescopeDrive(this.telescope, armState),
        new WristDrive(this.wrist, armState)
    ));
    // extensionPos = this.arm.extendPot.get();
    // if(this.arm.armState == true){
    //   SmartDashboard.putString("Game Piece", "Cone");
    //   if(Math.abs(originalPivot-armState[0])<30){
    //     SmartDashboard.putString("extending", "yes");
    //     new ParallelCommandGroup(
    //       new PivotDrive(armState),
    //       new TelescopeDrive(armState),
    //       new WristDrive(armState)
    //     );
    //     // new PivotDrive(armState);
    //     // new TelescopeDrive(armState);
    //     // new WristDrive(armState);
    //   }
    //   else{
    //     SmartDashboard.putString("extending", "no");

    //     new SequentialCommandGroup(
    //       new ParallelCommandGroup(
    //         new PivotDrive(armState),
    //         new WristDrive(armState)
    //       ),
    //       new TelescopeDrive(armState)
    //     );
    //   }
    // }
    // else{
    //   SmartDashboard.putString("Game Piece", "Cube");

    //   if(Math.abs(originalPivot-armState[3])<30){
    //     SmartDashboard.putString("arm moving", "yes");
    //     new ParallelCommandGroup(
    //       new PivotDrive(armState),
    //       new TelescopeDrive(armState),
    //       new WristDrive(armState)
    //     );
    //   }
    //   else{
    //     SmartDashboard.putString("arm moving", "no");

    //     new SequentialCommandGroup(
    //       new ParallelCommandGroup(
    //         new PivotDrive(armState),
    //         new WristDrive(armState)
    //       ),
    //       new TelescopeDrive(armState)
    //     );
    //   }
    // }
    // if(Math.abs(extensionPos-armState[1])<2){
    //   isFinished = true;
    // }
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
