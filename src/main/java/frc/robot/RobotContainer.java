// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AbsoluteTurn;
import frc.robot.commands.ArmCancelCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.Balance;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualPivotCommand;
import frc.robot.commands.ManualTelescopeCommand;
import frc.robot.commands.ManualWristCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.PivotHoldCommand;
import frc.robot.commands.TelescopeDrive;
import frc.robot.commands.ToggleIntake;
import frc.robot.commands.WristDrive;
// import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final static Drivetrain driveTrain = new Drivetrain();
  // private final Arm arm = new Arm();
  private final Wrist wrist = new Wrist();
  public final static Telescope telescope = new Telescope();
  public final static Pivot pivot = new Pivot();
  private final Claw claw = new Claw();




  //Joysticks
  public static Joystick driverL = new Joystick(0);
  public static Joystick driverR = new Joystick(1);
  public static Joystick operatorL = new Joystick(2);
  public static Joystick operatorR = new Joystick(3);

  public static JoystickButton operatorL1 = new JoystickButton(operatorL, 1);
  public static JoystickButton operatorL2 = new JoystickButton(operatorL, 2);
  public static JoystickButton operatorL3 = new JoystickButton(operatorL, 3);
  public static JoystickButton operatorL4 = new JoystickButton(operatorL, 4);

  public static JoystickButton operatorL5 = new JoystickButton(operatorL, 5);
  public static JoystickButton operatorL6 = new JoystickButton(operatorL, 6);
  public static JoystickButton operatorL7 = new JoystickButton(operatorL, 7);
  public static JoystickButton operatorL8 = new JoystickButton(operatorL, 8);
  public static JoystickButton operatorL9 = new JoystickButton(operatorL, 9);
  public static JoystickButton operatorL10 = new JoystickButton(operatorL, 10);

  public static JoystickButton operatorL11 = new JoystickButton(operatorL, 11);
  public static JoystickButton operatorL12 = new JoystickButton(operatorL, 12);
  public static JoystickButton operatorL13 = new JoystickButton(operatorL, 13);
  public static JoystickButton operatorL14 = new JoystickButton(operatorL, 14);
  public static JoystickButton operatorL15 = new JoystickButton(operatorL, 15);
  public static JoystickButton operatorL16 = new JoystickButton(operatorL, 16);


  public static JoystickButton operatorR1 = new JoystickButton(operatorR, 1);
  public static JoystickButton operatorR2 = new JoystickButton(operatorR, 2);
  public static JoystickButton operatorR3 = new JoystickButton(operatorR, 3);
  public static JoystickButton operatorR4 = new JoystickButton(operatorR, 4);
  public static JoystickButton operatorR8 = new JoystickButton(operatorR, 8);


  public static POVButton operatorLPovUpButton = new POVButton(operatorL, 0);
  public static POVButton operatorLPovDownButton = new POVButton(operatorL, 180);


  public static JoystickButton driverR5 = new JoystickButton(driverR, 5);
  public static JoystickButton driverR6 = new JoystickButton(driverR, 6);
  public static JoystickButton driverR7 = new JoystickButton(driverR, 7);
  public static JoystickButton driverR8 = new JoystickButton(driverR, 8);
  public static JoystickButton driverR9 = new JoystickButton(driverR, 9);
  public static JoystickButton driverR10 = new JoystickButton(driverR, 10);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    this.driveTrain.setDefaultCommand(new Drive(this.driveTrain));
    configureBindings();
  }
 
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // operatorR2.onTrue(new PivotDrive(this.arm, Constants.intakeHigh));

    //Outtake
    operatorR1.whileTrue(new IntakeCommand(claw, -0.8));
    // operatorR1.whileTrue(new ManualWristCommand(operatorR1, wrist));
    // operatorR2.onTrue(new WristDrive(wrist, Constants.outtakeMid, false).alongWith(new PivotHoldCommand(operatorR, pivot, Constants.outtakeMid, false, 0)).alongWith(new TelescopeDrive(telescope, Constants.outtakeMid)));
    // operatorR3.onTrue(new WristDrive(wrist, Constants.outtakeHigh, false).alongWith(new PivotHoldCommand(operatorR, pivot, Constants.outtakeHigh, false, 0)).alongWith(new TelescopeDrive(telescope, Constants.outtakeHigh)));
    // operatorR4.onTrue(new WristDrive(wrist, Constants.outtakeLow, false).alongWith(new PivotHoldCommand(operatorR, pivot, Constants.outtakeLow, false, 0)).alongWith(new TelescopeDrive(telescope, Constants.outtakeLow)));

    // //Intake
    operatorL1.whileTrue(new IntakeCommand(claw, 0.9));
    // operatorL3.onTrue(new WristDrive(wrist, Constants.intakeLow, false).alongWith(new PivotHoldCommand(operatorR, pivot, Constants.intakeLow, false, 0)).alongWith(new TelescopeDrive(telescope, Constants.intakeLow)));
    // operatorL2.onTrue(new WristDrive(wrist, Constants.intakeSide, false).alongWith(new PivotHoldCommand(operatorR, pivot, Constants.intakeSide, false, 0)).alongWith(new TelescopeDrive(telescope, Constants.intakeSide)));
    // operatorL4.onTrue(new WristDrive(wrist, Constants.intakeHigh, false).alongWith(new PivotHoldCommand(operatorR, pivot, Constants.intakeHigh, false, 0)).alongWith(new TelescopeDrive(telescope, Constants.intakeHigh)));

    // //Travel
    // operatorL11.onTrue(new WristDrive(wrist, Constants.travel, false).alongWith(new PivotHoldCommand(operatorR, pivot, Constants.travel, false, 0)).alongWith(new TelescopeDrive(telescope, Constants.travel)));
    // operatorL12.onTrue(new WristDrive(wrist, Constants.travel, false).alongWith(new PivotHoldCommand(operatorR, pivot, Constants.travel, false, 0)).alongWith(new TelescopeDrive(telescope, Constants.travel)));
    // operatorL13.onTrue(new WristDrive(wrist, Constants.travel, false).alongWith(new PivotHoldCommand(operatorR, pivot, Constants.travel, false, 0)).alongWith(new TelescopeDrive(telescope, Constants.travel)));
    // operatorL14.onTrue(new WristDrive(wrist, Constants.travel, false).alongWith(new PivotHoldCommand(operatorR, pivot, Constants.travel, false, 0)).alongWith(new TelescopeDrive(telescope, Constants.travel)));
    // operatorL15.onTrue(new WristDrive(wrist, Constants.travel, false).alongWith(new PivotHoldCommand(operatorR, pivot, Constants.travel, false, 0)).alongWith(new TelescopeDrive(telescope, Constants.travel)));
    // operatorL16.onTrue(new WristDrive(wrist, Constants.travel, false).alongWith(new PivotHoldCommand(operatorR, pivot, Constants.travel, false, 0)).alongWith(new TelescopeDrive(telescope, Constants.travel)));

    // //Manual Mode
    // // operatorL8.whileTrue(new ManualTelescopeCommand(operatorL, telescope).alongWith(new ManualPivotCommand(operatorR, pivot)).alongWith(new ManualWristCommand(operatorR, wrist)));    operatorL8.whileTrue(new ManualTelescopeCommand(operatorL, telescope).alongWith(new ManualPivotCommand(operatorR, pivot)).alongWith(new ManualWristCommand(operatorR, wrist)));
    operatorL8.whileTrue(new ManualTelescopeCommand(operatorL, telescope).alongWith(new ManualWristCommand(operatorR, wrist)).alongWith(new ManualPivotCommand(operatorR, pivot)));

    
    // //Balance
    // driverR5.whileTrue(new Balance(driveTrain));
    // driverR6.whileTrue(new Balance(driveTrain));
    // driverR7.whileTrue(new Balance(driveTrain));
    // driverR8.whileTrue(new Balance(driveTrain));
    // driverR9.whileTrue(new Balance(driveTrain));
    // driverR10.whileTrue(new Balance(driveTrain));


    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // if(Robot.arm.armState == true){
    //   opertorR8.onTrue(new PivotDrive(Constants.coneIntake));
    // }
    // else{
    //   opertorR8.onTrue(new Balance());

    // }
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    
    // return Autos.oneScoreEngageAuto(driverL, pivot, wrist, telescope, claw, driveTrain);
    return Autos.exampleAuto(m_exampleSubsystem);
    // return new ParallelCommandGroup(
    //   new PivotHoldCommand(operatorR, pivot, Constants.outtakeHigh),
    //   new WristDrive(wrist, Constants.outtakeHigh),
    //   new TelescopeDrive(telescope, Constants.outtakeHigh),
    //   new SequentialCommandGroup(
    //     new WaitCommand(3),
    //     new OuttakeCommand(claw, 0.5),
    //     new WaitCommand(2),
    //     new OuttakeCommand(claw, 0),
    //     new ParallelCommandGroup(
    //       new PivotHoldCommand(driverR, pivot, Constants.travel),
    //       new WristDrive(wrist, Constants.travel),
    //       new TelescopeDrive(telescope, Constants.travel)
    //     ),
    //     new DriveStraight(driveTrain, 0.5),
    //     new WaitCommand(5),
    //     new DriveStraight(driveTrain, 0)
    //   )
    // );
    // return Autos.oneScoreRightAuto(driveTrain, driverL, pivot, wrist, telescope);
    }
}
