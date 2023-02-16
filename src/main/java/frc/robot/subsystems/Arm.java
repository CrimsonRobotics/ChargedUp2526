// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  CANSparkMax arm;
  CANSparkMax extension;
  CANSparkMax wrist;
  AnalogPotentiometer armPot;
  AnalogPotentiometer extendPot;
  AnalogPotentiometer wristPot;
  PIDController armPID;
  PIDController wristPID;
  PIDController extendPID;
  DoubleSolenoid intakeSolenoid;



  public Arm() {
    arm = new CANSparkMax(Constants.armID, MotorType.kBrushless);
    wrist = new CANSparkMax(Constants.wristID, MotorType.kBrushless);
    extension = new CANSparkMax(Constants.extensionID, MotorType.kBrushless);

    wrist.setSmartCurrentLimit(5);


    armPot = new AnalogPotentiometer(1, 360, 0);

    wristPot = new AnalogPotentiometer(3, 360, 0);

    extendPot = new AnalogPotentiometer(2, 33.1, 0);

    armPID = new PIDController(Constants.armkP, Constants.armkI, Constants.armkD);
    armPID.setIntegratorRange(-Constants.armMaxPercent, Constants.armMaxPercent);

    wristPID = new PIDController(Constants.wristkP, Constants.wristkI, Constants.wristkD);
    wristPID.setIntegratorRange(-Constants.armMaxPercent, Constants.armMaxPercent);

    extendPID = new PIDController(Constants.extendkP, Constants.extendkI, Constants.extendkD);
    extendPID.setIntegratorRange(-Constants.armMaxPercent, Constants.armMaxPercent);

    intakeSolenoid = new DoubleSolenoid(
      Constants.PCM, 
      PneumaticsModuleType.CTREPCM, 
      Constants.intakeSolenodIDS[0], Constants.intakeSolenodIDS[1]);

  }

  public void ArmDrive(double moveSpeed) {
    arm.set(moveSpeed);
  }

  public void ExtendDrive(double moveSpeed) {
    extension.set(moveSpeed);
  }

  public void WristDrive(double moveSpeed) {
    wrist.set(moveSpeed);
  }

  public void MoveArm(double armcase[]) {
    SmartDashboard.putNumber("Arm Case",armcase[0]);

    double armPotReadout = Robot.arm.armPot.get();
    // double armPotReadout = 120;

    double armspeed = MathUtil.clamp(armPID.calculate(armPotReadout, armcase[0]), -Constants.armMaxPercent, Constants.armMaxPercent);
    armspeed = armspeed / 100;
    Robot.arm.ArmDrive(armspeed);
    SmartDashboard.putNumber("Arm Speed",armspeed);

    double wristPotReadout = Robot.arm.wristPot.get();
    // double wristPotReadout = 100;
    double wristspeed = MathUtil.clamp(wristPID.calculate(wristPotReadout, armcase[1]), -Constants.armMaxPercent, Constants.armMaxPercent);
    wristspeed = wristspeed / 100;
    Robot.arm.ArmDrive(wristspeed);
    SmartDashboard.putNumber("Wrist Speed",wristspeed);

    double extendPotReadout = Robot.arm.extendPot.get();
    // double extendPotReadout = 15;

    double extendspeed = MathUtil.clamp(extendPID.calculate(extendPotReadout, armcase[2]), -Constants.armMaxPercent, Constants.armMaxPercent);
    extendspeed = extendspeed / 100;
    Robot.arm.ArmDrive(extendspeed);
    SmartDashboard.putNumber("Extend Speed",extendspeed);

    if (armcase[3] == 0) {
      intakeSolenoid.set(Value.kForward);
      SmartDashboard.putString("Intake State", "Cone");

    } else if(armcase[3] == 1) {
      SmartDashboard.putString("Intake State", "Cube");
    }

  }

  @Override
  public void periodic() {
    RobotContainer container = Robot.m_robotContainer;
    SmartDashboard.putNumber("ArmPot readout", Robot.arm.armPot.get());
    SmartDashboard.putNumber("WristPot readout", Robot.arm.wristPot.get());
    SmartDashboard.putNumber("ExtendPot readout", Robot.arm.extendPot.get());

    // This method will be called once per scheduler run
    if(container.driverR.getRawButton(1) == true){
      MoveArm(Constants.coneIntake);
      SmartDashboard.putBoolean("Arm On", true);

    }
    else if(container.driverR.getRawButton(2) == true){
      MoveArm(Constants.cubeIntake);
      SmartDashboard.putBoolean("Arm On", true);


    }
    else{
      SmartDashboard.putBoolean("Arm On", false);

    }


  
  }
}
