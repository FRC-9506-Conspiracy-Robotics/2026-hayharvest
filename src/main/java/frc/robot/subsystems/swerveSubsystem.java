// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.dyn4j.geometry.Rotation;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.swerveConstants;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

public class swerveSubsystem extends SubsystemBase {
  /** Creates a new swerveSubsystem. */
  //All of this is taken from nessie's swerve code
  private final SwerveDrive swerveDrive;

  public swerveSubsystem(File directory) {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.POSE;

    try{
      swerveDrive = new SwerveParser(directory).createSwerveDrive(
        Constants.swerveConstants.maxDriveSpeed,
        new Pose2d(
          new Translation2d(
            Meter.of(1),
            Meter.of(4)
          ),
          Rotation2d.fromDegrees(0)
        )
      );
    } catch (Exception e){
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false);
    swerveDrive.setCosineCompensator(false);
    swerveDrive.setAngularVelocityCompensation(true, true, -0.13);
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
    setupPathPlanner();
  }

  public SwerveDrive getSwerveDrive(){
    return swerveDrive;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setupPathPlanner(){
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
      final boolean enableFeedForward = true;
      AutoBuilder.configure(
        swerveDrive::getPose,
        swerveDrive::resetOdometry, 
        swerveDrive::getRobotVelocity,
        (speedRobotRelative, moduleFeedFowards) -> {
          if (enableFeedForward) {
            swerveDrive.drive(
              speedRobotRelative, 
              swerveDrive.kinematics.toSwerveModuleStates(speedRobotRelative), 
              moduleFeedFowards.linearForces()
            );
          } else {
            swerveDrive.setChassisSpeeds(speedRobotRelative);
          }
        },
        new PPHolonomicDriveController(
          new PIDConstants(5, 0.0, 0.0),
          new PIDConstants(5, 0.0,0.0)
        ),
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this
      );
    } catch (Exception e){
      e.printStackTrace();
    }
    PathfindingCommand.warmupCommand().schedule();
  }

  public Command getAutonomousCommand(String pathName){
    return new PathPlannerAuto(pathName);
  }

  public Command driveToPose(Pose2d pose) {
    PathConstraints constraints = new PathConstraints(
      swerveDrive.getMaximumChassisVelocity(),
      4.0,
      swerveDrive.getMaximumChassisAngularVelocity(),
      Units.degreesToRadians(720)
    );
    return AutoBuilder.pathfindToPose(
      pose,
      constraints,
      edu.wpi.first.units.Units.MetersPerSecond.of(0)
      );
  }

  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
      SwerveDriveTest.setDriveSysIdRoutine(
        new Config(),
        this, 
        swerveDrive, 
        12,
        true
      ),
      3.0, 5.0, 3.0
    );
  }

  public Command sysIdAngleMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
      SwerveDriveTest.setAngleSysIdRoutine(
        new Config(),
        this,
        swerveDrive
      ),
      3.0, 5.0, 3.0
    );
  }

  public Command centerModulesComman() {
    return run(() -> Arrays.asList(swerveDrive.getModules()).forEach(it -> it.setAngle(0.0)));
  }

  public void replaceSwerveModuleFeedforward(double kS, double kV, double kA) {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(
      scaledInputs.getX(),
      scaledInputs.getY(),
      angle.getRadians(),
      getHeading().getRadians(),
      swerveConstants.maxDriveSpeed
    );
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds){
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void drive(ChassisSpeeds velocity){
    swerveDrive.drive(velocity);
  }

  //moves the bot
  public void drive(Translation2d translation, double rotation, boolean fieldrelative){
    swerveDrive.drive(translation, rotation, fieldrelative, false);
  }

  public void driveFieldOriented(ChassisSpeeds velocity){
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity){
    return run(
      () -> {
        swerveDrive.driveFieldOriented(velocity.get());
      }
    );
  }

  public void lock() {
    swerveDrive.lockPose();
  }

  public void zeroGyro(){
    swerveDrive.zeroGyro();
  }

  public Command zero(){
    return runOnce(() -> zeroGyro());
  }

  //moves robot based on translation and angle
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX){
    return run(() -> {
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
        translationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
        translationY.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity()), 0.8
      ),
        Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(), true, false
      );
    });
  }

  //drives a certain distance and stops using pose
  public Command driveToDistanceCommand(double distanceinMeters, double speedInMetersPerSecond){
    return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0))).until(() -> swerveDrive.getPose().getTranslation().getDistance(new Translation2d(0, 0)) > distanceinMeters);
  }
}