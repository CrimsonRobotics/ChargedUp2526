// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Drivetrain;
// import frc.robot.subsystems.LED;

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

  static double testkP;
  static double testkI;
  static double testkD;
  static double testMaxPercent;
  static double testSetPoint;


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

    
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

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
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // pivotTest = SmartDashboard.getNumber("Pivot Position", 180);
    // telescopeTest = SmartDashboard.getNumber("Telescope Position", 30);
    // wristTest = SmartDashboard.getNumber("Wrist Position", 263);
    // testArray[0] = pivotTest;
    // testArray[1] = wristTest;
    // testArray[2] = telescopeTest;
    // testArray[3] = pivotTest;
    // testArray[4] = wristTest;
    // testArray[5] = telescopeTest;

    // testkP = SmartDashboard.getNumber("testkP", 0.75);
    // testkI = SmartDashboard.getNumber("testkI", 0);
    // testkD = SmartDashboard.getNumber("testkD", 0.28);
    // testMaxPercent = SmartDashboard.getNumber("testMaxPercent", 0.3);
    // testSetPoint = SmartDashboard.getNumber("testSetPoint", 0);


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
