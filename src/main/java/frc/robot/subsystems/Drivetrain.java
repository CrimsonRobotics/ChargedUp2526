// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANAnalog.AnalogMode;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;


public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  CANSparkMax frontLeft;
  CANSparkMax frontRight;
  RelativeEncoder encoderLeft;

  public ADXRS450_Gyro gyro;

  public PIDController turnPID;
  public PIDController alignPID;
  public PigeonIMU pigeon;
  public DigitalInput limitSwitch;
  public SparkMaxAnalogSensor thePot;
  public SparkMaxLimitSwitch theLimit;
  // public SparkMaxAnalogSensor theLimit;
  // public AnalogPotentiometer pot;
  // public AnalogPotentiometer pot2;


  public Drivetrain() {
    frontLeft = new CANSparkMax(Constants.fLID, MotorType.kBrushless);
    frontRight = new CANSparkMax(Constants.fRID, MotorType.kBrushless);

    frontLeft.setInverted(true);
    frontRight.setInverted(false);

    frontLeft.setIdleMode(IdleMode.kBrake);
    frontRight.setIdleMode(IdleMode.kBrake);

    gyro = new ADXRS450_Gyro();
    gyro.calibrate();
    pigeon = new PigeonIMU(10); //Need to check device number

    turnPID = new PIDController(Constants.turnkP, Constants.turnkI, Constants.turnkD);
    turnPID.setIntegratorRange(-Constants.pidMaxPercent, Constants.pidMaxPercent);

    alignPID = new PIDController(Constants.alignkP, Constants.alignkI, Constants.alignkD);
    alignPID.setIntegratorRange(-Constants.alignMaxPercent, Constants.alignMaxPercent);

    limitSwitch = new DigitalInput(0);
    thePot = frontRight.getAnalog(Mode.kAbsolute);
    // theLimit = frontRight.getAnalog(Mode.kAbsolute);
    theLimit = frontRight.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    encoderLeft = frontLeft.getEncoder();
    encoderLeft.setPositionConversionFactor(0.04);
    // pot = new AnalogPotentiometer(0, 10 /*10*/, 0);
    // pot2 = new AnalogPotentiometer(3, 3600, 0);

  }

  public void TeleopDrive(double forwardSpeed, double turnSpeed) {

    frontLeft.set(forwardSpeed - turnSpeed);
    frontRight.set(forwardSpeed + turnSpeed);

  }

  public void ManualDrive(double leftSpeed, double rightSpeed) {
    frontLeft.set(leftSpeed);
    frontRight.set(rightSpeed);
  }

  public void Align(){
    double gyroReadout = Robot.driveTrain.gyro.getAngle();
    double pitchReadout = Robot.driveTrain.pigeon.getPitch();
    double yawReadout = Robot.driveTrain.pigeon.getYaw();

      if (yawReadout>5){
        Robot.driveTrain.ManualDrive(-0.1 , 0.1);
      }

      else if (yawReadout<-5) {
        Robot.driveTrain.ManualDrive(0.1 , -0.1);
      }

      else {
        Robot.driveTrain.ManualDrive(0, 0);
      }
  }

  public void pidAlign() {
   
    double gyroReadout = Robot.driveTrain.gyro.getAngle() % 360;
    double speed = MathUtil.clamp(turnPID.calculate(gyroReadout, Constants.turnSetpoint), -Constants.pidMaxPercent, Constants.pidMaxPercent);
    speed = speed / 100;
    Robot.driveTrain.TeleopDrive(0, speed);
    SmartDashboard.putNumber("Speed",speed);

  }

  public void balance() {

    // double gyroReadout = Robot.driveTrain.gyro.getAngle() % 360;
    double pigeonReadout = Robot.driveTrain.pigeon.getRoll() % 360;

    double speed = MathUtil.clamp(alignPID.calculate(pigeonReadout, Constants.alignSetpoint), -Constants.alignMaxPercent, Constants.alignMaxPercent);
    speed = speed / 100;
    Robot.driveTrain.TeleopDrive(speed, 0);
    SmartDashboard.putNumber("align speed",speed);

  }

  public void pigeonAlign() {
  
    // double[] ypr = new double [3];
    // Robot.driveTrain.pigeon.getYawPitchRoll(ypr);
    // double gyroReadout = ypr[0]; //% 360;
    double pigeonReadout = Robot.driveTrain.pigeon.getYaw() % 360;
    SmartDashboard.putNumber("Gyro Heading", pigeonReadout);

    double speed = MathUtil.clamp(turnPID.calculate(pigeonReadout, Constants.turnSetpoint), -Constants.pidMaxPercent, Constants.pidMaxPercent);
    speed = speed / 100;
    Robot.driveTrain.TeleopDrive(0, -speed);
    // Robot.driveTrain.ManualDrive(-speed, speed);

    SmartDashboard.putNumber("pigeonAlignSpeed", speed);

  }
  
  @Override
  public void periodic() {
      // This method will be called once per scheduler run
      // double[] ypr = new double [3];
      // Robot.driveTrain.pigeon.getYawPitchRoll(ypr);
      // double gyroReadout = ypr[0]; 
      // SmartDashboard.putNumber("yaw", gyroReadout);
      SmartDashboard.putData(turnPID);
      SmartDashboard.putBoolean("limit switch", !Robot.driveTrain.limitSwitch.get());
      SmartDashboard.putBoolean("the limit switch", theLimit.isPressed());

      // SmartDashboard.putNumber("pot", Robot.driveTrain.pot.get());
      // SmartDashboard.putNumber("pot2", Robot.driveTrain.pot2.get());
      SmartDashboard.putNumber("testing", Constants.turnkP);
      SmartDashboard.putNumber("fl rel encoder", encoderLeft.getPosition());
      SmartDashboard.putNumber(" fl encoder position", encoderLeft.getPositionConversionFactor());
      // SmartDashboard.putNumber("the pot", thePot.getPosition()/3.29*10);
      SmartDashboard.putNumber("the pot", thePot.getPosition()/3.26);


      if (!Robot.driveTrain.limitSwitch.get()){
        SmartDashboard.putString("limit", "true");
      }
      else{
        SmartDashboard.putString("limit", "false");
      }
      // SmartDashboard.putNumber("Yaw", Robot.driveTrain.pigeon.getYaw());
    
      RobotContainer container = Robot.m_robotContainer;
      if(container.driverL.getRawButton(1) == true && container.driverL.getRawButton(2) == false){
        // Robot.driveTrain.pidAlign();
        Robot.driveTrain.balance();
        // Robot.driveTrain.Align();
      }
      else if(container.driverL.getRawButton(2) == true && container.driverL.getRawButton(1) == false){
        
        Robot.driveTrain.pigeonAlign();
        SmartDashboard.putString("pigeonAligning", "true");
        // Robot.driveTrain.ManualDrive(0.1, 0.1);
  
      }
      // else if(container.driverL.getRawButton(2) == true && container.driverL.getRawButton(1) == true){
        
      //   Robot.driveTrain.pigeonAlign();
      //   Robot.driveTrain.balance();
      //   SmartDashboard.putString("both align", "true");

      //   // Robot.driveTrain.ManualDrive(0.1, 0.1);
  
      // }
      else if(container.driverL.getRawButton(3) == true){
        // Robot.driveTrain.gyro.reset();
        Robot.driveTrain.pigeon.setYaw(0);
        // Robot.driveTrain.pigeon.setAccumZAngle(0);
        // Robot.driveTrain.pigeon.setYaw(0);

      }
      else{

        Robot.driveTrain.ManualDrive(0, 0);
        SmartDashboard.putString("pigeonAligning", "false");
        SmartDashboard.putString("both align", "falae");


      }
    
  }
}
