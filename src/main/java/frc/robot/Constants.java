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

  public static int[] leftIDs = {55,33,35}; //33
  public static int[] rightIDs = {30,41,60}; //32

  //arm motors
  public static int pivotIDs[] = {42, 48}; //36 //46
  public static int telescopeID = 38;
  public static int wristID = 40;
  public static int intakeID = 39;


  public final static int PCM = 0;
  public final static int[] clawSolenoidIDs = {4,5};
  public final static int[] shiftSolenoidIDs = {0,2};
  public final static int[] brakeSolenoidIDs = {1,3};

  public static double intakeSpeed = 0.6;
  public static double outtakeSpeed = -0.5;

  public static double driveExpo = 1;
  public static double turnExpo = 1;

  public static double[] intakeLow = {/*Cone*/20, 20, 20,/*Cube */ 50,50,50};
  public static double[] intakeSide = {/*Cone*/20, 20, 20,/*Cube */ 50,50,50};
  public static double[] intakeShelf = {/*Cone*/20, 20, 20,/*Cube */ 50,50,50};
  public static double[] intakeHigh = {/*Cone*/30, 30, 30,/*Cube */ 40,40,40};
  public static double[] outtakeLow = {/*Cone*/0, 0, 0,/*Cube */ 10,10,10};
  public static double[] outtakeMid = {/*Cone*/40, 40, 40,/*Cube */ 20,20,20};
  public static double[] outtakeHigh = {/*Cone*/60, 60, 60,/*Cube */ 30,30,30};
  public static double[] travel = {/*Cone*/189, 20, 3.26,/*Cube */ 189, 20, 3.26};


  // public static double[] cubeIntake = {20, 20, 20, 1};

  public static double alignError = 2;
  public static double balanceError = 5;
  public static double pivotError = 2;
  public static double telescopeError = 2;
  public static double wristError = 2;

  public static double extendStopDistance = 30;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
