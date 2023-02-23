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
  //drivetrain motors
  public static int fLID = 60;
  public static int fRID = 59;

  public static int[] leftIDs = {59,60,40};
  public static int[] rightIDs = {41,42,43};

  //arm motors
  public static int pivotID = 30;
  public static int extensionID = 31;
  public static int wristID = 32;

  public final static int PCM = 0;
  public final static int[] intakeSolenodIDS = {4,2};

  public static double driveExpo = 1;
  public static double turnExpo = 1;

  public static double[] coneIntake = {0, 80, 0, 0};
  public static double[] cubeIntake = {20, 20, 20, 1};


  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
