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

  //arm motors
  public static int armID = 30;
  public static int extensionID = 31;
  public static int wristID = 32;

  public final static int PCM = 0;
  public final static int[] intakeSolenodIDS = {4,2};

  public static double driveExpo = 1;
  public static double turnExpo = 1;

  public static double turnkP = 0.6; //0.5
  public static double turnkI = 0.; //0.01;
  public static double turnkD = 0; //0.02;
  public static double turnSetpoint = 0;
  public static double pidMaxPercent = 90;

  public static double alignkP = 0.7; //1
  public static double alignkI = 0;
  public static double alignkD = 0;
  public static double alignSetpoint = -3.69;
  public static double alignMaxPercent = 70;

  public static double armkP = 0.7; //1
  public static double armkI = 0;
  public static double armkD = 0;
  public static double armSetpoint = -3.69;
  public static double armMaxPercent = 70;

  public static double wristkP = 0.7; //1
  public static double wristkI = 0;
  public static double wristkD = 0;

  public static double extendkP = 0.7; //1
  public static double extendkI = 0;
  public static double extendkD = 0;

  public static double[] coneIntake = {0, 80, 0, 0};
  public static double[] cubeIntake = {20, 20, 20, 1};


  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
