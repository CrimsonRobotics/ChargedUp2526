// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;


public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  // CANSparkMax frontLeft;
  // CANSparkMax frontRight;

  CANSparkMax left1;
  CANSparkMax left2;
  CANSparkMax left3;
  CANSparkMax right1;
  CANSparkMax right2;
  CANSparkMax right3;

  public Drivetrain() {
    // frontLeft = new CANSparkMax(Constants.fLID, MotorType.kBrushless);
    // frontRight = new CANSparkMax(Constants.fRID, MotorType.kBrushless);

    // frontLeft.setInverted(true);
    // frontRight.setInverted(false);

    // frontLeft.setIdleMode(IdleMode.kBrake);
    // frontRight.setIdleMode(IdleMode.kBrake);

    left1 = new CANSparkMax(Constants.leftIDs[0], MotorType.kBrushless);
    left2 = new CANSparkMax(Constants.leftIDs[1], MotorType.kBrushless);
    left3 = new CANSparkMax(Constants.leftIDs[2], MotorType.kBrushless);
    right1 = new CANSparkMax(Constants.rightIDs[0], MotorType.kBrushless);
    right2 = new CANSparkMax(Constants.rightIDs[1], MotorType.kBrushless);
    right3 = new CANSparkMax(Constants.rightIDs[2], MotorType.kBrushless);

    left1.setInverted(false);
    left2.setInverted(false);
    left3.setInverted(false);
    right1.setInverted(false);
    right2.setInverted(false);
    right3.setInverted(false);

    left1.setIdleMode(IdleMode.kBrake);
    left2.setIdleMode(IdleMode.kBrake);
    left3.setIdleMode(IdleMode.kBrake);
    right1.setIdleMode(IdleMode.kBrake);
    right2.setIdleMode(IdleMode.kBrake);
    right3.setIdleMode(IdleMode.kBrake);

    left1.setSmartCurrentLimit(Constants.driveLim);
    left2.setSmartCurrentLimit(Constants.driveLim);
    left3.setSmartCurrentLimit(Constants.driveLim);
    right1.setSmartCurrentLimit(Constants.driveLim);
    right2.setSmartCurrentLimit(Constants.driveLim);
    right3.setSmartCurrentLimit(Constants.driveLim);
  }

  public void TeleopDrive(double forwardSpeed, double turnSpeed) {

    left1.set(forwardSpeed - turnSpeed);
    left2.set(forwardSpeed - turnSpeed);
    left3.set(forwardSpeed - turnSpeed);
    right1.set(forwardSpeed + turnSpeed);
    right2.set(forwardSpeed + turnSpeed);
    right3.set(forwardSpeed + turnSpeed);


  }

  public void ManualDrive(double leftSpeed, double rightSpeed) {
    left1.set(leftSpeed);
    left2.set(leftSpeed);
    left3.set(leftSpeed);
    right1.set(rightSpeed);
    right2.set(rightSpeed);
    right3.set(rightSpeed);
  }

  
  @Override
  public void periodic() {
      SmartDashboard.putNumber("Left 1 Current", left1.getOutputCurrent());
      SmartDashboard.putNumber("Left 2 Current", left2.getOutputCurrent());
      SmartDashboard.putNumber("Left 3 Current", left3.getOutputCurrent());
      SmartDashboard.putNumber("Left 4 Current", right1.getOutputCurrent());
      SmartDashboard.putNumber("Left 5 Current", right2.getOutputCurrent());
      SmartDashboard.putNumber("Left 6 Current", right3.getOutputCurrent());

      // SmartDashboard.putNumber("Left 1 Voltage", left1.getBusVoltage());
      // SmartDashboard.putNumber("Left 2 Voltage", left2.getBusVoltage());
      // SmartDashboard.putNumber("Left 3 Voltage", left3.getBusVoltage());
      // SmartDashboard.putNumber("Left 4 Voltage", right1.getBusVoltage());
      // SmartDashboard.putNumber("Left 5 Voltage", right2.getBusVoltage());
      // SmartDashboard.putNumber("Left 6 Voltage", right3.getBusVoltage());

      // SmartDashboard.putNumber("Left 1 Temp", left1.getMotorTemperature());
      // SmartDashboard.putNumber("Left 2 Temp", left2.getMotorTemperature());
      // SmartDashboard.putNumber("Left 3 Temp", left3.getMotorTemperature());
      // SmartDashboard.putNumber("Left 4 Temp", right1.getMotorTemperature());
      // SmartDashboard.putNumber("Left 5 Temp", right2.getMotorTemperature());
      // SmartDashboard.putNumber("Left 6 Temp", right3.getMotorTemperature());
    
  }
}
