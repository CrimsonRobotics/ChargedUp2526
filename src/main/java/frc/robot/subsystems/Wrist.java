// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */

  CANSparkMax wristTest;
  public Wrist() {

    wristTest = new CANSparkMax(60, MotorType.kBrushless);

    wristTest.setSmartCurrentLimit(10);
    // wristTest.setSecondaryCurrentLimit(5);

  }

  public void WristDrive(double moveSpeed) {
    wristTest.set(moveSpeed);
  }

  @Override
  public void periodic() {
    
    RobotContainer container = Robot.m_robotContainer;
    SmartDashboard.putNumber("Wrist Current", wristTest.getOutputCurrent());
    SmartDashboard.putNumber("Wrist Volatage", wristTest.getBusVoltage());
    SmartDashboard.putNumber("Wrist temp", wristTest.getMotorTemperature());

    // This method will be called once per scheduler run
    if(container.driverL.getRawButton(5) == true){
      Robot.wrist.WristDrive(1);
    }
    else if(container.driverL.getRawButton(4) == true){
      Robot.wrist.WristDrive(-1);
    }
    else if(container.driverL.getRawButton(6) == true){
      Robot.wrist.WristDrive(0.1);
    }
    else if(container.driverL.getRawButton(7) == true){
      Robot.wrist.WristDrive(-0.1);
    }
    else {
      Robot.wrist.WristDrive(0);
    }
  }
}
