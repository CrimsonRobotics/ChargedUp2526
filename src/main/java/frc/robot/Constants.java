// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  //turn sesitivity adjustment
  public static double joystickturnsens = 0.87;
  //drivetrain motors
  // public static int fLID = 41; //60;
  // public static int fRID = 42;

  public static int[] leftIDs = {33,41,35}; //33
  public static int[] rightIDs = {30,49,60}; //32

  public static int driveLim = 45;

  //arm motors
  public static int pivotIDs[] = {46, 48}; //36 //46
  public static int telescopeID = 38;
  public static int wristID = 40;
  public static int intakeID = 39;


  public final static int PCM = 0;
  public final static int[] clawSolenoidIDs = {1,3};
  public final static int[] shiftSolenoidIDs = {0,2};
  public final static int[] brakeSolenoidIDs = {4,5};

  public static double intakeSpeed = 0.6;
  public static double outtakeSpeed = -0.5;

  public static double driveExpo = 2.24;//2
  public static double turnExpo = 1;

  public static double[] intakeLow = {/*Cone*/93, 320, 15,/*Cube */ 80, 292,11};
  // public static double[] intakeLow = {/*Cone*/79, 290, 30,/*Cube */ 79, 294,36};

  public static double[] intakeSide = {/*Cone*/62, 225, 10,/*Cube */ 62,225, 10};
  // public static double[] intakeShelf = {/*Cone*/20, 263, 20,/*Cube */ 50,50,50};
  public static double[] intakeHigh = {/*Cone*/160, 335, 55,/*Cube */ 155,333, 53};
  public static double[] intakeFarShelf = {/*Cone*/147, 338, 97,/*Cube */ 150,333, 90};


  public static double[] outtakeLow = {/*Cone*/260, 200, 15,/*Cube */ 260,230,15};
  public static double[] outtakeMid = {/*Cone*/227, 180, 85,/*Cube */ 240,250,15};
  public static double[] outtakeHigh = {/*Cone*/227, 190, 205,/*Cube */ 230, 190, 155}; //207
  public static double[] travel = {/*Cone*/192, 320, 10,/*Cube */ 192, 320, 10};
  // public static double[] test = {/*Cone*/186, 76, 60,/*Cube */ 186, 76, 60};
//{/*Cone*/ Pivot, Wrist, Telescope, /*Cube */ Pivot, Wrist, Telescope}


  // public static double[] cubeIntake = {20, 20, 20, 1};

  public static double alignError = 2;
  public static double balanceError = 5;
  public static double pivotError = 2;
  public static double telescopeError = 2;
  public static double wristError = 2;

  public static double extendStopDistance = 40;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
