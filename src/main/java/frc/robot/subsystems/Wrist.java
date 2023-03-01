// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Wrist extends SubsystemBase {
  CANSparkMax wrist;
  public SparkMaxAnalogSensor wristPot;
  public PIDController wristPID;
  /** Creates a new Wrist. */
  public Wrist() {
    wrist = new CANSparkMax(Constants.wristID, MotorType.kBrushed);

    wrist.setIdleMode(IdleMode.kBrake);
    
    // wristPot = new AnalogPotentiometer(3, 360, 0);
    wristPot = wrist.getAnalog(Mode.kAbsolute);

    wristPID = new PIDController(PIDConstants.pivotkP, PIDConstants.pivotkI, PIDConstants.pivotkD);
    wristPID.setIntegratorRange(-PIDConstants.pivotMaxPercent, PIDConstants.pivotMaxPercent);
  }

  public void WristDrive(double moveSpeed) {
    wrist.set(moveSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    RobotContainer container = Robot.m_robotContainer;

    SmartDashboard.putNumber("WristPot readout", wristPot.getVoltage());

  }
}
