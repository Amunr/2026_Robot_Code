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
    public static final double deadband = 0.1;
  }

  public static class climberConstants{
    public static final int climberMotorID=16;
    public static final int deployMotorID = 17;
    public static final double forwardRotations = 10.0;
    public static final double forwardRotationsClimb = 10.0;
    public static final double climbLevel1 = 10.0;
    public static final double climbLevel2 = 20.0;
    public static final double climbLevel3 = 30.0;
    public static final double overShoot = 2.0;
  }
  public static class intakeConstants{
    public static final int intakeMotorID=14;
    public static final int intakeDeployMotorID=15;
    public static final double intakeOutRotations = 10.0;
    public static final double setIntakeSpeed = 0.1;
    
  }
   public static class reefConstants {
    public static final double reefX = 4.491799831390381;
    public static final double reefY = 4.026460647583008;
    public static final double[] pointsX = { 5.8339008113657025 , 5.668800811365703 , 4.326700000000001 , 3.1496991886342975 , 3.3147991886342965 , 4.6568999999999985 , 5.668800811365703 , 5.8339008113657025 , 4.6569 , 3.3147991886342973 , 3.149699188634296 , 4.326699999999998};
    public static final double[] pointsY = {4.61068120583519 , 3.15627720583519 , 2.572056 , 3.44223879416481 , 4.89664279416481 , 5.480864 , 4.896642794164811 , 3.442238794164811 , 2.572056 , 3.1562772058351882 , 4.6106812058351885 , 5.480864};

    }
}
