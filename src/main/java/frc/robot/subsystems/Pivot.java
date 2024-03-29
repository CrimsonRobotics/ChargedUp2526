// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;


public class Pivot extends SubsystemBase {
  CANSparkMax pivot1;
  CANSparkMax pivot2;
  public static AnalogPotentiometer pivotPot;
  public PIDController pivotPID;
  public LinearFilter filter;
  public static boolean armState;
  public SparkMaxLimitSwitch limitFront;
  public SparkMaxLimitSwitch limitBack;


  /** Creates a new Pivot. */
  public Pivot() {
    pivot1 = new CANSparkMax(Constants.pivotIDs[0], MotorType.kBrushless);
    pivot2 = new CANSparkMax(Constants.pivotIDs[1], MotorType.kBrushless);

    pivot1.restoreFactoryDefaults();
    pivot2.restoreFactoryDefaults();

    pivot1.setSmartCurrentLimit(60);
    pivot2.setSmartCurrentLimit(60);

    // pivot1.setIdleMode(IdleMode.kBrake);
    // pivot2.setIdleMode(IdleMode.kBrake);

    pivot1.setIdleMode(IdleMode.kCoast);
    pivot2.setIdleMode(IdleMode.kCoast);

    pivot1.setInverted(false);
    pivot2.setInverted(false);
    
    pivotPot = new AnalogPotentiometer(1, 360, 0);

    limitFront = pivot1.getForwardLimitSwitch(Type.kNormallyOpen);
    limitBack = pivot1.getReverseLimitSwitch(Type.kNormallyOpen);

    pivotPID = new PIDController(PIDConstants.pivotkP, PIDConstants.pivotkI, PIDConstants.pivotkD);
    pivotPID.setIntegratorRange(-PIDConstants.pivotMaxPercent, PIDConstants.pivotMaxPercent);
    // pivotPID.setTolerance(2, 20);
    filter = LinearFilter.singlePoleIIR(0.1, 0.02);

    armState = true;

  }

  public void PivotDrive(double moveSpeed) {
    pivot1.set(moveSpeed);
    pivot2.set(moveSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    RobotContainer container = Robot.m_robotContainer;
    SmartDashboard.putNumber("PivotPot readout", pivotPot.get());
    // SmartDashboard.putBoolean("Forward Limit", limitFront.isPressed());
    // SmartDashboard.putBoolean("Reverse Limit", limitBack.isPressed());

    // SmartDashboard.putNumber("pivto1 current", pivot1.getOutputCurrent());
    // SmartDashboard.putNumber("pivtot2 current", pivot2.getOutputCurrent());

    // SmartDashboard.putNumber("Pivot pos error", pivotPID.getPositionError());
    // SmartDashboard.putNumber("Pivot vel error", pivotPID.getVelocityError());



    // if(container.operatorR.getRawButton(7)){
    //   PivotDrive(0.3);
    // }
    // else{
    //   PivotDrive(0);
    // }

    if(container.operatorR.getRawButton(14)||container.operatorR.getRawButton(15)||container.operatorR.getRawButton(16)){
      armState = true;
    }
    else if(container.operatorR.getRawButton(11)||container.operatorR.getRawButton(12)||container.operatorR.getRawButton(13)){
      armState = false;
    }
  }
}
