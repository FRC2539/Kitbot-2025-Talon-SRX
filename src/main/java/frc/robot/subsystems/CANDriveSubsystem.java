// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.revrobotics.spark.SparkBase.PersistMode;
//import com.revrobotics.spark.SparkBase.ResetMode;
//import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.config.SparkMaxConfig;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
// com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

// Class to drive the robot over CAN
public class CANDriveSubsystem extends SubsystemBase {
private final WPI_TalonSRX leftLeader;
private final WPI_TalonSRX leftFollower;
private final WPI_TalonSRX rightLeader;
private final WPI_TalonSRX rightFollower;

private final DifferentialDrive drive;

//@SuppressWarnings("deprecation")
public CANDriveSubsystem() {
  // create brushed motors for drive
  leftLeader = new WPI_TalonSRX(DriveConstants.LEFT_LEADER_ID);
  leftFollower = new WPI_TalonSRX(DriveConstants.LEFT_FOLLOWER_ID);
  rightLeader = new WPI_TalonSRX(DriveConstants.RIGHT_LEADER_ID);
  rightFollower = new WPI_TalonSRX(DriveConstants.RIGHT_FOLLOWER_ID);
  // --- Configure Left Side Motors ---
  // Set the left follower to follow the left leader
  leftFollower.follow(leftLeader);

  // Set the neutral mode (what the motor does when no power is applied)
  leftLeader.setNeutralMode(NeutralMode.Brake);
  leftFollower.setNeutralMode(NeutralMode.Brake);

  // Invert the left side motors if necessary.
  // This depends on how your motors are mounted and wired.
  leftLeader.setInverted(false); // Adjust as needed
  leftFollower.setInverted(false); // Adjust as needed, usually same as leader

  // Configure a feedback device (e.g., encoder) for the leader motor if you plan to use PID control.
  leftLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10); // PID Loop 0, 10ms timeout

  // --- Configure Right Side Motors ---
  // Set the right follower to follow the right leader
  rightFollower.follow(rightLeader);

  // Set the neutral mode for the right side motors
  rightLeader.setNeutralMode(NeutralMode.Brake);
  rightFollower.setNeutralMode(NeutralMode.Brake);

  // Invert the right side motors if necessary.
  // Often, one side needs to be inverted relative to the other for differential drive.
  rightLeader.setInverted(true); // Adjust as needed (often true if left is false)
  rightFollower.setInverted(true); // Adjust as needed, usually same as leader

  // Configure a feedback device for the right leader motor
  rightLeader.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10); // PID Loop 0, 10ms timeout


  // Initialize DifferentialDrive with the left and right motor groups.
  // WPILib's DifferentialDrive handles the inversion of one side automatically
  // if you pass them in the correct order (left, right).
  // However, since we've already set inversion on the individual Talons,
  // ensure the overall robot movement is correct.
  drive = new DifferentialDrive(leftLeader, rightLeader);

  // Set safety for the DifferentialDrive. This is good practice.
  drive.setSafetyEnabled(true);
  drive.setExpiration(0.1); // How long motors will run without new input (seconds)
  drive.setMaxOutput(.80); // Maximum output to the motors (1.0 is full power)

  }

  @Override
  public void periodic() {
  }

  // sets the speed of the drive motors
  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }
}
