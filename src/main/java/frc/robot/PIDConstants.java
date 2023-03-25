// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class PIDConstants {
    public static double turnkP = 0.6; //0.5
    public static double turnkI = 0; //0.01;
    public static double turnkD = 0; //0.02;
    public static double turnSetpoint = 0;
    public static double pidMaxPercent = 90;

    // public static double turnkP = Robot.testkP; //0.5
    // public static double turnkI = Robot.testkI; //0.01;
    // public static double turnkD = Robot.testkD; //0.02;
    // public static double turnSetpoint = Robot.testSetPoint;
    // public static double pidMaxPercent = Robot.testMaxPercent;
  
    public static double balancekP = 0.65; //1
    public static double balancekI = 0.01;
    public static double balancekD = 0;
    public static double balanceSetpoint = 0;
    public static double balanceMaxPercent = 100;

    // public static double balancekP = Robot.testkP; //1
    // public static double balancekI = Robot.testkI;
    // public static double balancekD = Robot.testkD;
    // public static double balanceSetpoint = Robot.testSetPoint;
    // public static double balanceMaxPercent = Robot.testMaxPercent;

    public static double straightkP = 0.6;
    public static double straightkI = 0;
    public static double straightkD = 0;
    public static double straightSetpoint = 0;
    public static double straightMaxPercent = 100;

    // public static double straightkP = Robot.testkP;
    // public static double straightkI = Robot.testkI;
    // public static double straightkD = Robot.testkD;
    // public static double straightSetpoint = Robot.testSetPoint;
    // public static double straightMaxPercent = Robot.testMaxPercent;
  
    public static double pivotkP = 1;//0.8; //1
    public static double pivotkI = 0;//0.05;
    public static double pivotkD = 0.1;//0.2;
    public static double pivotMaxPercent = 100;


    // public static double pivotkP = Robot.testkP;
    // public static double pivotkI = Robot.testkI;
    // public static double pivotkD = Robot.testkD;
    // public static double pivotMaxPercent = Robot.testMaxPercent;

  
    public static double wristkP = 3; //1
    public static double wristkI = 0;
    public static double wristkD = 0;
    public static double wristMaxPercent = 100;

    // public static double wristkP = Robot.testkP; //1
    // public static double wristkI = Robot.testkI;
    // public static double wristkD = Robot.testkD;
    // public static double wristMaxPercent = Robot.testMaxPercent;

  
    public static double telescopekP = 6.7; //1
    public static double telescopekI = 0;
    public static double telescopekD = 0;
    public static double telescopeMaxPercent = 100;

    // public static double telescopekP = Robot.testkP; //1
    // public static double telescopekI = Robot.testkI;
    // public static double telescopekD = Robot.testkD;
    // public static double telescopeMaxPercent = Robot.testMaxPercent;

    public static double downkP = 0.7; //1
    public static double downkI = 0;
    public static double downkD = 0;
    public static double downMaxPercent = 50;

    // public static double downkP = Robot.testkP; //1
    // public static double downkI = Robot.testkI;
    // public static double downkD = Robot.testkD;
    // public static double downMaxPercent = Robot.testMaxPercent;

}
