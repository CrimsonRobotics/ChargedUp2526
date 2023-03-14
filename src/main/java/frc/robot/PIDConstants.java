// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class PIDConstants {
    public static double turnkP = 0.6; //0.5
    public static double turnkI = 0.; //0.01;
    public static double turnkD = 0; //0.02;
    public static double turnSetpoint = 0;
    public static double pidMaxPercent = 90;
  
    public static double balancekP = 0.7; //1
    public static double balancekI = 0;
    public static double balancekD = 0;
    public static double balanceSetpoint = 0;
    public static double balanceMaxPercent = 80;

    public static double straightkP = 0.6;
    public static double straightkI = 0;
    public static double straightkD = 0;
    public static double straightSetpoint = 0;
    public static double straightMaxPercent = 100;
  
    public static double pivotkP = 0.75;//0.8; //1
    public static double pivotkI = 0;//0.05;
    public static double pivotkD = 0.28;//0.2;
    // public static double pivotSetpoint = 0.39;
    public static double pivotMaxPercent = 30;
  
    public static double wristkP = 3; //1
    public static double wristkI = 0;
    public static double wristkD = 0;
    public static double wristMaxPercent = 100;

  
    public static double telescopekP = 6.7; //1
    public static double telescopekI = 0;
    public static double telescopekD = 0;
    public static double telescopeMaxPercent = 100;

    public static double downkP = 0.7; //1
    public static double downkI = 0;
    public static double downkD = 0;
    public static double downMaxPercent = 100;

}
