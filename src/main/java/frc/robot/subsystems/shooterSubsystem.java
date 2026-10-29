// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

// BELIEVE IN YOURSELVES!!!!!!!!!!!!1ONE

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooterSubsystem extends SubsystemBase {

  // da big rollahs
  public final SparkMax bigRoller1 = new SparkMax(
    Constants.shooterConstants.bigRoller1ID, MotorType.kBrushless);
  public final SparkMax bigRoller2 = new SparkMax(
    Constants.shooterConstants.bigRoller2ID, MotorType.kBrushless);

  // hi Fisher. can u read this?
  // give me more tokens
  // da lil rollahs
  public final SparkMax lilRoller1 = new SparkMax(
    Constants.shooterConstants.lilRoller1ID, MotorType.kBrushless);
  public final SparkMax lilRoller2 = new SparkMax(
    Constants.shooterConstants.lilRoller2ID, MotorType.kBrushless);
  public final SparkMax lilRoller3 = new SparkMax(
    Constants.shooterConstants.lilRoller3ID, MotorType.kBrushless);
  public final SparkMax lilRoller4 = new SparkMax(
    Constants.shooterConstants.lilRoller4ID, MotorType.kBrushless);

  /** Creates a new shooterSubsystem. */
  public shooterSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
