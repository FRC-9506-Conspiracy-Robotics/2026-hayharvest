// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class deployIntake extends Command {
  /** Creates a new deployIntake. */
  public intakeSubsystem intake;

  //THESE ERRORS WILL DISAPPEAR ONCE WE HAVE THE BOT

  public deployIntake(intakeSubsystem _intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = _intake;
    this.addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.intake.desiredPosition = !this.intake.desiredPosition;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double deploySpeed = 0.01;

    if(this.intake.desiredPosition == this.intake.intakeDeployed){
      this.intake.deployLeaderMotor.set(-deploySpeed);
    } else{
      this.intake.deployLeaderMotor.set(deploySpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.intake.deployLeaderMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    double deployPosition; //measure once the bot is built
    double stowedPosition; //measure once the bot is built

    if(this.intake.desiredPosition == this.intake.intakeDeployed){
      return this.intake.deployEncoder.getPosition() < deployPosition;
    } else {
      return this.intake.deployEncoder.getPosition() > stowedPosition;
    }

  }
}
