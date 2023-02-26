// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.PIDConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Telescope extends SubsystemBase {
  /** Creates a new Telesscope. */
  CANSparkMax telescope;
  public AnalogPotentiometer telescopePot;
  public PIDController telescopePID;
  
  public Telescope() {
    telescope = new CANSparkMax(Constants.telescopeID, MotorType.kBrushless);
    telescopePot = new AnalogPotentiometer(2, 33.1, 0);

    telescope.setIdleMode(IdleMode.kBrake);

    telescopePID = new PIDController(PIDConstants.telescopekP, PIDConstants.telescopekI, PIDConstants.telescopekD);
    telescopePID.setIntegratorRange(-PIDConstants.telescopeMaxPercent, PIDConstants.telescopeMaxPercent);
  }

  public void telescopeDrive(double moveSpeed) {
    telescope.set(moveSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    RobotContainer container = Robot.m_robotContainer;
    SmartDashboard.putNumber("telescopePot readout", telescopePot.get());
    

  }
}
