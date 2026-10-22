// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DeployIntakeCommand;

public class intakeSubsystem extends SubsystemBase {
  public final SparkMax deployLeaderMotor = new SparkMax(Constants.intakeConstants.intakeDeployID, MotorType.kBrushless); //it won't error if I change the ID
  public final SparkMax deployFollowerMotor = new SparkMax(Constants.intakeConstants.intakeFollowerID, MotorType.kBrushless); //same as leader
  public final SparkMax intakeMotor = new SparkMax(Constants.intakeConstants.intakeID, MotorType.kBrushless); //same as the rest

  public final RelativeEncoder deployEncoder = deployLeaderMotor.getEncoder();
  public final RelativeEncoder followerEncoder = deployFollowerMotor.getEncoder();

  public boolean desiredPosition = false; //I will fix the error after I write the deployintake commandd

  public boolean intakeDeployed = true;
  public boolean intakeStowed = false;

  public double deploySpeed = 0.01;

  // final DoublePublisher deployInfo;
  SparkMaxConfig intakeConfig = new SparkMaxConfig();
  SparkMaxConfig deployLeaderConfig = new SparkMaxConfig();
  SparkMaxConfig deployFollowerConfig = new SparkMaxConfig();
 

  /** Creates a new intakeSubsystem. */
  public intakeSubsystem() {
      intakeConfig
    .idleMode(IdleMode.kCoast)
    .smartCurrentLimit(0);// CHANGE THIS 

      deployLeaderConfig
    .idleMode(IdleMode.kCoast)
    .smartCurrentLimit(0); //CHANGE THIS
    deployLeaderMotor.configure(deployLeaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

 deployFollowerConfig
    .idleMode(IdleMode.kCoast)
    .follow(deployLeaderMotor, true);
    deployFollowerMotor.configure(deployFollowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public Command deployIntake(){
    return new DeployIntakeCommand(this);//I will create the command so it doesn't error anymore
  }

  public Command startIntakeCommand(){
    return startEnd(
      () -> intakeMotor.set(-1),
      () -> intakeMotor.set(0)
    );
  }

  public Command toggleDeploy() {
    return runOnce(
      () -> {this.desiredPosition = !this.desiredPosition;
            this.deploySpeed = 0.01;
      }
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
