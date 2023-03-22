// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.LED;
import frc.robot.subsystems.Pivot;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static RobotContainer m_robotContainer;

  public static double startuporiginalYaw;

  static double[] testArray = {/*Cone*/180, 263, 30,/*Cube */ 180, 263, 30};

  static double pivotTest;
  static double telescopeTest;
  static double wristTest;

  DigitalOutput led0;
  DigitalOutput led1;
  DigitalOutput led2;
  DigitalOutput led3;

  // DigitalInput ledTest;


  static double testkP;
  static double testkI;
  static double testkD;
  static double testMaxPercent;
  static double testSetPoint;

  

  // public AddressableLED led;
  // public AddressableLEDBuffer ledBuffer;

  // public static double[] defaultPID = {/*Cone*/186, 76, 30,/*Cube */ 186, 76, 30};

  // public static Drivetrain driveTrain;
  // public static LED led;
  // public static Arm arm;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    // driveTrain = new Drivetrain();
    m_robotContainer = new RobotContainer();
    // led = new LED();
    // arm = new Arm();
    // driveTrain.gyro.calibrate();
    // driveTrain.pigeon.setYaw(0);
    // driveTrain.pigeon.set
    RobotContainer.driveTrain.pigeon.setYaw(0);
    CameraServer.startAutomaticCapture("Intake Camera", 0).setResolution(102, 77);
    // CameraServer.startAutomaticCapture("Outtake Camera", 1);
    
    // led = new AddressableLED(9);

    led0 = new DigitalOutput(11);
    led1 = new DigitalOutput(13);
    led2 = new DigitalOutput(15);
    led3 = new DigitalOutput(17);

    // led0.set(true);
    // led1.set(true);
    // led2.set(true);
    // led3.set(true);
    // led0 = new DigitalInput(11);

    // ledBuffer = new AddressableLEDBuffer(58);
    // led.setLength(ledBuffer.getLength());

    // // Set the data
    // led.setData(ledBuffer);
    // led.start();

    // SmartDashboard.putNumberArray("PID Positions", defaultPID);
    // SmartDashboard.putNumber("Pivot Position", 180);
    // SmartDashboard.putNumber("Telescope Position", 30);
    // SmartDashboard.putNumber("Wrist Position", 180);

    // SmartDashboard.putNumber("testkP", 0.75);
    // SmartDashboard.putNumber("testkI", 0);
    // SmartDashboard.putNumber("testkD", 0.28);
    // SmartDashboard.putNumber("testMaxPercent", 0.3);
    // SmartDashboard.putNumber("testSetPoint", 0);



  }

  // public void SetColor(int r, int g, int b){
  //   for (var i = 0; i < ledBuffer.getLength(); i++) {
  //     // Sets the specified LED to the RGB values for red
  //     ledBuffer.setRGB(i, r, g, b);
  //  }

  //  led.setData(ledBuffer);

  // }

  // private void rainbow(int c) {
  //   int rainbowFirstPixelHue = c;
  //   // For every pixel
  //   for (var i = 0; i < ledBuffer.getLength(); i++) {
  //     // Calculate the hue - hue is easier for rainbows because the color
  //     // shape is a circle so only one value needs to precess
  //     final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
  //     // Set the value
  //     ledBuffer.setHSV(i, hue, 255, 128);
  //   }
  //   // Increase by to make the rainbow "move"
  //   rainbowFirstPixelHue += 3;
  //   // Check bounds
  //   rainbowFirstPixelHue %= 180;

  //   led.setData(ledBuffer);
  // }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    // SmartDashboard.putNumber("Yaw Heading", driveTrain.pigeon.getYaw());
    // SmartDashboard.putNumber("Pitch Heading", driveTrain.pigeon.getPitch());
    // SmartDashboard.putNumber("Roll Heading", driveTrain.pigeon.getRoll());
    // led0.set(true);
    // led1.set(true);
    // led2.set(true);
    // led3.set(true);
    
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // led1.set(true);
    led0.set(true);
    led1.set(true);
    led2.set(true);
    led3.set(true);

  }

  @Override
  public void disabledPeriodic() {
    // SetColor(0, 0, 0);
    // led1.set(true);
    led0.set(true);
    led1.set(true);
    led2.set(true);
    led3.set(true);

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    startuporiginalYaw = Drivetrain.pigeon.getYaw();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // rainbow(10);
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    led0.set(true);
    led1.set(true);
    led2.set(true);
    led3.set(true);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    led0.set(true);
    led1.set(true);
    led2.set(true);
    led3.set(true);
    // pivotTest = SmartDashboard.getNumber("Pivot Position", 180);
    // telescopeTest = SmartDashboard.getNumber("Telescope Position", 30);
    // wristTest = SmartDashboard.getNumber("Wrist Position", 263);
    // testArray[0] = pivotTest;
    // testArray[1] = wristTest;
    // testArray[2] = telescopeTest;
    // testArray[3] = pivotTest;
    // testArray[4] = wristTest;
    // testArray[5] = telescopeTest;
    // led1.set(true);

    // testkP = SmartDashboard.getNumber("testkP", 0.75);
    // testkI = SmartDashboard.getNumber("testkI", 0);
    // testkD = SmartDashboard.getNumber("testkD", 0.28);
    // testMaxPercent = SmartDashboard.getNumber("testMaxPercent", 0.3);
    // // testSetPoint = SmartDashboard.getNumber("testSetPoint", 0);
    // SetColor(0, 255, 0)
    // if (Pivot.armState){
    //   // SetColor(255, 255, 0);
    //   led0.set(true);
    //   led1.set(true);
    //   led2.set(true);
    //   led3.set(true);

    // }
    // else if(Pivot.armState == false){
    //   // SetColor(255, 0, 255);
    //   led0.set(true);
    //   led1.set(true);
    //   led2.set(true);
    //   led3.set(true);
    // }
    // else {
    //   SetColor(0, 255, 0);
    // }

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
