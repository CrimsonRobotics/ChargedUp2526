// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  CANSparkMax intakeMotor;
  public DoubleSolenoid intakeSolenoid;

  public Claw() {
    intakeMotor = new CANSparkMax(Constants.intakeID, MotorType.kBrushless);

    intakeSolenoid = new DoubleSolenoid(
      Constants.PCM, 
      PneumaticsModuleType.CTREPCM, 
      Constants.clawSolenoidIDs[0], Constants.clawSolenoidIDs[1]);
  }

  public void IntakeDrive(double moveSpeed) {
    intakeMotor.set(moveSpeed);
  }

  public void ToggleIntake(boolean state){
    if (state == true){
      SmartDashboard.putString("Game Piece", "Cone");
      intakeSolenoid.set(Value.kForward);
    }
    else {
      SmartDashboard.putString("Game Piece", "Cube");
      intakeSolenoid.set(Value.kReverse);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ToggleIntake(Pivot.armState);
    SmartDashboard.putBoolean("Arm State", Pivot.armState);

  }
}
