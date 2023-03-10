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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.PIDConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;


public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */

  CANSparkMax frontLeft;
  CANSparkMax frontRight;
  RelativeEncoder encoderLeft;

  CANSparkMax left1;
  CANSparkMax left2;
  CANSparkMax left3;
  CANSparkMax right1;
  CANSparkMax right2;
  CANSparkMax right3;

  // public ADXRS450_Gyro gyro;

  public PIDController turnPID;
  public PIDController balancePID;
  public PIDController straightPID;
  public static PigeonIMU pigeon;
  // public DigitalInput limitSwitch;
  // public SparkMaxAnalogSensor thePot;
  // public SparkMaxLimitSwitch theLimit;
  public DoubleSolenoid shifter;
  public DoubleSolenoid parkingBrake;


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
    right1.setInverted(true);
    right2.setInverted(true);
    right3.setInverted(true);

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

    // gyro = new ADXRS450_Gyro();
    // gyro.calibrate();
    pigeon = new PigeonIMU(10); //Need to check device number

    turnPID = new PIDController(PIDConstants.turnkP, PIDConstants.turnkI, PIDConstants.turnkD);
    turnPID.setIntegratorRange(-PIDConstants.pidMaxPercent, PIDConstants.pidMaxPercent);

    balancePID = new PIDController(PIDConstants.balancekP, PIDConstants.balancekI, PIDConstants.balancekD);
    balancePID.setIntegratorRange(-PIDConstants.balanceMaxPercent, PIDConstants.balanceMaxPercent);

    straightPID = new PIDController(PIDConstants.straightkP, PIDConstants.straightkI, PIDConstants.straightkD);
    straightPID.setIntegratorRange(-PIDConstants.straightMaxPercent, PIDConstants.straightMaxPercent);
          
    // limitSwitch = new DigitalInput(0);
    // thePot = right1.getAnalog(Mode.kAbsolute);
    
    // thePot = frontRight.getAnalog(Mode.kAbsolute);
    // theLimit = frontRight.getAnalog(Mode.kAbsolute);
    // theLimit = frontRight.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    // encoderLeft = frontLeft.getEncoder();
    encoderLeft = left1.getEncoder();
    encoderLeft.setPositionConversionFactor(0.04);
    // pot = new AnalogPotentiometer(0, 10 /*10*/, 0);
    // pot2 = new AnalogPotentiometer(3, 3600, 0);

    shifter = new DoubleSolenoid(
      Constants.PCM, 
      PneumaticsModuleType.CTREPCM, 
      Constants.shiftSolenoidIDs[0], Constants.shiftSolenoidIDs[1]);

    parkingBrake = new DoubleSolenoid(
      Constants.PCM, 
      PneumaticsModuleType.CTREPCM, 
      Constants.brakeSolenoidIDs[0], Constants.brakeSolenoidIDs[1]);

  }

  // public void TeleopDrive(double forwardSpeed, double turnSpeed) {

  //   frontLeft.set(forwardSpeed - turnSpeed);
  //   frontRight.set(forwardSpeed + turnSpeed);

  // }

  // public void ManualDrive(double leftSpeed, double rightSpeed) {
  //   frontLeft.set(leftSpeed);
  //   frontRight.set(rightSpeed);
  // }

  public void TeleopDrive(double forwardSpeed, double turnSpeed) {

    left1.set(forwardSpeed - turnSpeed);
    left2.set(forwardSpeed - turnSpeed);
    left3.set(forwardSpeed - turnSpeed);
    right1.set(forwardSpeed + turnSpeed);
    right2.set(forwardSpeed + turnSpeed);
    right3.set(forwardSpeed + turnSpeed);


  }

  // public void shift(Joystick driverL) {
  //   if (driverL.getRawButton(11) ||
  //       driverL.getRawButton(12) ||
  //       driverL.getRawButton(13) ||
  //       driverL.getRawButton(14) ||
  //       driverL.getRawButton(15) ||
  //       driverL.getRawButton(16)) {
  //     shifter.set(Value.kForward);
  //     // Robot.currentState = "Shifting";
  //   }

  //   else {
  //     shifter.set(Value.kReverse);
  //   }
  // }

  // public void park(Joystick l, Joystick r){
  //   if(l.getRawButton(8)&&l.getRawButton(14)){
  //     parkingBrake.set(Value.kForward);
  //     SmartDashboard.putString("parking", "parked");
  //   }
  //   else if(r.getRawButton(8)&&r.getRawButton(14)){
  //     parkingBrake.set(Value.kReverse);
  //     SmartDashboard.putString("parking", "not parked");

  //   }
  // }

  public void ManualDrive(double leftSpeed, double rightSpeed) {
    left1.set(leftSpeed);
    left2.set(leftSpeed);
    left3.set(leftSpeed);
    right1.set(rightSpeed);
    right2.set(rightSpeed);
    right3.set(rightSpeed);
  }

  // public void Align(){
  //   double gyroReadout = Robot.driveTrain.gyro.getAngle();
  //   double pitchReadout = Robot.driveTrain.pigeon.getPitch();
  //   double yawReadout = Robot.driveTrain.pigeon.getYaw();

  //     if (yawReadout>5){
  //       Robot.driveTrain.ManualDrive(-0.1 , 0.1);
  //     }

  //     else if (yawReadout<-5) {
  //       Robot.driveTrain.ManualDrive(0.1 , -0.1);
  //     }

  //     else {
  //       Robot.driveTrain.ManualDrive(0, 0);
  //     }
  // }

  // public void pidAlign() {
   
  //   double gyroReadout = Robot.driveTrain.gyro.getAngle() % 360;
  //   double speed = MathUtil.clamp(turnPID.calculate(gyroReadout, Constants.turnSetpoint), -Constants.pidMaxPercent, Constants.pidMaxPercent);
  //   speed = speed / 100;
  //   Robot.driveTrain.TeleopDrive(0, speed);
  //   SmartDashboard.putNumber("Speed",speed);

  // }

  // public void balance() {

  //   // double gyroReadout = Robot.driveTrain.gyro.getAngle() % 360;
  //   double pigeonReadout = Robot.driveTrain.pigeon.getRoll() % 360;

  //   double speed = MathUtil.clamp(balancePID.calculate(pigeonReadout, PIDConstants.balanceSetpoint), -PIDConstants.balanceMaxPercent, PIDConstants.balanceMaxPercent);
  //   speed = speed / 100;
  //   Robot.driveTrain.TeleopDrive(speed, 0);
  //   SmartDashboard.putNumber("align speed",speed);

  // }

  // public void pigeonAlign() {
  
  //   // double[] ypr = new double [3];
  //   // Robot.driveTrain.pigeon.getYawPitchRoll(ypr);
  //   // double gyroReadout = ypr[0]; //% 360;
  //   double pigeonReadout = Robot.driveTrain.pigeon.getYaw() % 360;
  //   SmartDashboard.putNumber("Gyro Heading", pigeonReadout);

  //   double speed = MathUtil.clamp(turnPID.calculate(pigeonReadout, PIDConstants.turnSetpoint), -PIDConstants.pidMaxPercent, PIDConstants.pidMaxPercent);
  //   speed = speed / 100;
  //   Robot.driveTrain.TeleopDrive(0, -speed);
  //   // Robot.driveTrain.ManualDrive(-speed, speed);

  //   SmartDashboard.putNumber("pigeonAlignSpeed", speed);

  // }
  
  @Override
  public void periodic() {
      RobotContainer container = Robot.m_robotContainer;

      // This method will be called once per scheduler run
      // double[] ypr = new double [3];
      // Robot.driveTrain.pigeon.getYawPitchRoll(ypr);
      // double gyroReadout = ypr[0]; 
      // SmartDashboard.putNumber("yaw", gyroReadout);
      SmartDashboard.putNumber("Left 1 Current", left1.getOutputCurrent());
      SmartDashboard.putNumber("Left 2 Current", left2.getOutputCurrent());
      SmartDashboard.putNumber("Left 3 Current", left3.getOutputCurrent());
      SmartDashboard.putNumber("Right 1 Current", right1.getOutputCurrent());
      SmartDashboard.putNumber("Right 2 Current", right2.getOutputCurrent());
      SmartDashboard.putNumber("Right 3 Current", right3.getOutputCurrent());

      SmartDashboard.putNumber("gyro", pigeon.getYaw());
      SmartDashboard.putNumber("gyro roll", pigeon.getRoll());
      SmartDashboard.putNumber("gyro pitch", pigeon.getPitch());



      // shift(container.driverL);
      // park(container.driverL, container.driverR);
      if(container.driverL.getRawButton(5) ||
        container.driverL.getRawButton(6) ||
        container.driverL.getRawButton(7) ||
        container.driverL.getRawButton(8) ||
        container.driverL.getRawButton(9) ||
        container.driverL.getRawButton(10)){
          
        parkingBrake.set(Value.kForward);
        SmartDashboard.putString("parking", "parked");
      }
      else if(container.driverR.getRawButton(11) ||
        container.driverR.getRawButton(12) ||
        container.driverR.getRawButton(13) ||
        container.driverR.getRawButton(14) ||
        container.driverR.getRawButton(15) ||
        container.driverR.getRawButton(16)){

        parkingBrake.set(Value.kReverse);
        SmartDashboard.putString("parking", "not parked");
  
      }

      if (container.driverL.getRawButton(5) ||
        container.driverL.getRawButton(6) ||
        container.driverL.getRawButton(7) ||
        container.driverL.getRawButton(8) ||
        container.driverL.getRawButton(9) ||
        container.driverL.getRawButton(10) ||
        container.driverL.getRawButton(11) ||
        container.driverL.getRawButton(12) ||
        container.driverL.getRawButton(13) ||
        container.driverL.getRawButton(14) ||
        container.driverL.getRawButton(15) ||
        container.driverL.getRawButton(16)) {
      shifter.set(Value.kForward);
      SmartDashboard.putString("shift state", "low gear");

      // Robot.currentState = "Shifting";
    }

    else {
      shifter.set(Value.kReverse);
      SmartDashboard.putString("shift state", "high gear");

    }
    System.out.print(shifter.get()); //shifter.get();


    SmartDashboard.putNumber("pov button", container.operatorL.getPOV());

      SmartDashboard.putNumber("Left 1 Voltage", left1.getBusVoltage());
      SmartDashboard.putNumber("Left 2 Voltage", left2.getBusVoltage());
      SmartDashboard.putNumber("Left 3 Voltage", left3.getBusVoltage());
      SmartDashboard.putNumber("Right 1 Voltage", right1.getBusVoltage());
      SmartDashboard.putNumber("Right 2 Voltage", right2.getBusVoltage());
      SmartDashboard.putNumber("Right 3 Voltage", right3.getBusVoltage());

      // SmartDashboard.putNumber("Left 1 Temp", left1.getMotorTemperature());
      // SmartDashboard.putNumber("Left 2 Temp", left2.getMotorTemperature());
      // SmartDashboard.putNumber("Left 3 Temp", left3.getMotorTemperature());
      // SmartDashboard.putNumber("Right 1 Temp", right1.getMotorTemperature());
      // SmartDashboard.putNumber("Right 2 Temp", right2.getMotorTemperature());
      // SmartDashboard.putNumber("Right 1 Temp", right3.getMotorTemperature());

      // SmartDashboard.putData(turnPID);
      // SmartDashboard.putBoolean("limit switch", !Robot.driveTrain.limitSwitch.get());
      // SmartDashboard.putBoolean("the limit switch", theLimit.isPressed());

      // // SmartDashboard.putNumber("pot", Robot.driveTrain.pot.get());
      // // SmartDashboard.putNumber("pot2", Robot.driveTrain.pot2.get());
      // SmartDashboard.putNumber("testing", Constants.turnkP);
      // SmartDashboard.putNumber("fl rel encoder", encoderLeft.getPosition());
      // SmartDashboard.putNumber(" fl encoder position", encoderLeft.getPositionConversionFactor());
      // SmartDashboard.putNumber("the pot", thePot.getPosition()/3.29*10);

      // if (!Robot.driveTrain.limitSwitch.get()){
      //   SmartDashboard.putString("limit", "true");
      // }
      // else{
      //   SmartDashboard.putString("limit", "false");
      // }
      // // SmartDashboard.putNumber("Yaw", Robot.driveTrain.pigeon.getYaw());
    
      // if(container.driverL.getRawButton(1) == true && container.driverL.getRawButton(2) == false){
      //   // Robot.driveTrain.pidAlign();
      //   Robot.driveTrain.balance();
      //   // Robot.driveTrain.Align();
      // }
      // else if(container.driverL.getRawButton(2) == true && container.driverL.getRawButton(1) == false){
        
      //   Robot.driveTrain.pigeonAlign();
      //   SmartDashboard.putString("pigeonAligning", "true");
      //   // Robot.driveTrain.ManualDrive(0.1, 0.1);
  
      // }
      // // else if(container.driverL.getRawButton(2) == true && container.driverL.getRawButton(1) == true){
        
      // //   Robot.driveTrain.pigeonAlign();
      // //   Robot.driveTrain.balance();
      // //   SmartDashboard.putString("both align", "true");

      // //   // Robot.driveTrain.ManualDrive(0.1, 0.1);
  
      // // }
      // else if(container.driverL.getRawButton(3) == true){
      //   // Robot.driveTrain.gyro.reset();
      //   Robot.driveTrain.pigeon.setYaw(0);
      //   // Robot.driveTrain.pigeon.setAccumZAngle(0);
      //   // Robot.driveTrain.pigeon.setYaw(0);

      // }
      // else{

      //   Robot.driveTrain.ManualDrive(0, 0);
      //   SmartDashboard.putString("pigeonAligning", "false");
      //   SmartDashboard.putString("both align", "falae");


      // }
  }
}
