// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.AnalogPotentiometer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Robot;

// public class LED extends SubsystemBase {
//   /** Creates a new LED. */
//   public AnalogPotentiometer ledPot;

//   public LED() {
//     // ledPot = new AnalogPotentiometer(0, 3600 /*10*/, 0);

//   }

//   public void ledSignal() {

//     int position = (int) Math.floor(Robot.led.ledPot.get()%360 / 60);
 
//     switch(position) {
//       case 0:
//         SmartDashboard.putNumber("pos", 1);
//         break;
//       case 1:
//         SmartDashboard.putNumber("pos", 2);
//         break;
//       case 2:
//         SmartDashboard.putNumber("pos", 3);
//         break;
//       case 3:
//         SmartDashboard.putNumber("pos", 4);
//         break;
//       case 4:
//         SmartDashboard.putNumber("pos", 5);
//         break;
//       case 5:
//         SmartDashboard.putNumber("pos", 6);
//         break;

//     }

//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     // SmartDashboard.putNumber("led position", Math.floor(Robot.led.ledPot.get()%360 / 60));
//     ledSignal();

//   }
// }
