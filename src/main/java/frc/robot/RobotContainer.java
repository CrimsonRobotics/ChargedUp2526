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
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.PivotDrive;
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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
  private final Drivetrain driveTrain = new Drivetrain();
  // private final Arm arm = new Arm();
  private final Wrist wrist = new Wrist();
  private final Telescope telescope = new Telescope();
  private final Pivot pivot = new Pivot();
  private final Claw claw = new Claw();




  //Joysticks
  public static Joystick driverL = new Joystick(0);
  public static Joystick driverR = new Joystick(1);
  public static Joystick operatorL = new Joystick(2);
  public static Joystick operatorR = new Joystick(3);

  public static JoystickButton lowOuttake = new JoystickButton(operatorR, 1);
  public static JoystickButton midOuttake = new JoystickButton(operatorR, 2);
  public static JoystickButton highOuttake = new JoystickButton(operatorR, 3);
  public static JoystickButton lowIntake = new JoystickButton(operatorR, 4);
  public static JoystickButton killArm = new JoystickButton(operatorR, 7);



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
    lowOuttake.onTrue(new PivotDrive(this.pivot, Constants.intakeLow));
    // midOuttake.onTrue(new PivotDrive(this.arm, Constants.intakeHigh));

    midOuttake.onTrue(new WristDrive(this.wrist, Constants.intakeLow));
    highOuttake.onTrue(new TelescopeDrive(this.telescope, Constants.intakeLow));
    // lowIntake.onTrue(new ArmDrive(this.arm, Constants.intakeLow));
    lowIntake.onTrue(new WristDrive(wrist, Constants.intakeLow).alongWith(new TelescopeDrive(telescope, Constants.intakeLow)).alongWith(new PivotDrive(pivot, Constants.intakeLow)));

    killArm.onTrue(new ArmCancelCommand(pivot, telescope, wrist));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // if(Robot.arm.armState == true){
    //   lowIntake.onTrue(new PivotDrive(Constants.coneIntake));
    // }
    // else{
    //   lowIntake.onTrue(new Balance());

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
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
