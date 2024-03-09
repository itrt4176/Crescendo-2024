// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double ROBOT_MASS = (85) * 0.453592; //32lbs * kg per pound

  public static final double LOOP_TIME  = 0.13;
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);



  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double TURN_CONSTANT    = 6.0;

    public static final double LEFT_DEADBAND_X  = 0.1;
    public static final double LEFT_DEADBAND_Y  = 0.1;
    public static final double RIGHT_DEADBAND_X = 0.1;
  }


  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds

    public static final class DriveConstants {
      public static final double P = 0.003;
      public static final double I = 0.0;
      public static final double D = 0.0;
      public static final double F = 0.0;
    }

    public static final class AngularConstants {
      public static final double P = 0.014225;
      public static final double I = 0.0;
      public static final double D = 0.22325;
      public static final double F = 0.00051;
    }
  }

  public static class  IntakeConstants {
    public static final int INTAKE_MAIN = 14; //placeholder
    public static final int INTAKE_FOLLOW = 15;//placeholder
    public static final int SHARP = 0; // placeholder
    
  }

  public static class ShooterConstants {
    public static final int MAIN_SHOOTER = 17;
    public static final int SUB_SHOOTER = 18;

    public static final double SPEAKER_SHOT_SPEED = -.65;
  }


  public static class ClimberConstants{
    public static final int FORWARD_LIMIT_DIO = 0;
    public static final int REVERSE_LIMIT_DIO = 1;
    public static final double FLIPPER_ROTATIONS_TO_DEGREES = -1.0 * 360.0 / (100.0 * (22.0 / 16.0)); //VERIFY!!!
  }
}
