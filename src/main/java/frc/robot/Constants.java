// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }


  public static class  IntakeConstants {
    public static final int INTAKE_MAIN = 14; //placeholder
    public static final int INTAKE_FOLLOW = 15;//placeholder
    public static final int SHARP = 0; // placeholder
    
  }

  public static class ShooterConstants {
    public static final int MAIN_SHOOTER = 17; //placeholder
    public static final int SUB_SHOOTER = 18; //placeholder

    public static final double SPEAKER_SHOT_SPEED = .75;
  }


  public static class ClimberConstants{
    public static final int WINCH_MAIN = 25;
    public static final int WINCH_FOLLOW = 26;
    public static final double FLIPPER_ROTATIONS_TO_DEGREES = 1 * 360 / (50 * (22 / 16)); //VERIFY!!!
    public static final double WINCH_ROTATIONS_TO_DEGREES = 1 * 360 / 20; //VERIFY!!!
  }
}
