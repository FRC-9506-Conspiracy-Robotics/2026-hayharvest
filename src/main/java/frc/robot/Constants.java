// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class intakeConstants{
      public static int intakeID; //will be edited
      public static int intakeDeployID; //edit
      public static int intakeFollowerID; //edit
    }

  public static class shooterConstants{
    public static int bigRoller1ID;
    public static int bigRoller2ID;

    public static int lilRoller1ID;
    public static int lilRoller2ID;
    public static int lilRoller3ID;
    public static int lilRoller4ID;
  }

  public static class swerveConstants{
    public static final double maxDriveSpeed = Units.feetToMeters(15);
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadband = 0.05;
  }
}
