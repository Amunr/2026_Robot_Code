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
    public static final int kOperatorControllerPort = 1;
    public static final double deadband = 0.1;
  }

public static class climberConstants{
    public static final int climberMotorID=17;
    public static final int deployMotorID = 16;
    public static final double forwardRotations = 0.0;
    public static final double forwardRotationsClimb = 0.0;
    public static final double upCyc = 0;
    public static final double downCyc = 0;
    public static final double pushoutU= 48.0;
    public static final double pushoutD = 48.0;
    public static final double maxLevel = 48.0;
    public static final double minLevel = 0.0;
    public static final double climbLevel3 = 0.0;
    public static final double overShoot = 0.0;
    public static  double climberP = 0.11;
    public static  double climberI = 0.000001;
    public static  double climberD = 0.001;

    
        public static final int hookDeploy = 3;  
        public static final int hookDeployIn = 3;  

        public static final int climberPushoutN = 4;
        public static final int climberPushoutIn = 4;
  }
  public static class intakeConstants{
    public static final int intakeMotorID=15;
    public static final int intakeMotorSecond = 19;
    public static final int intakeNuIDLeftF= 15;
        public static final int intakeNuIDLeftR= 14;

    // public static final int intakeNuIDRightF= 2;
    //     public static final int intakeNuIDRightR= 2;
    public static final int pnumaticID = 18;
    public static final double intakeOutRotations = 22.0;
    public static final double intakeInRotations = 2.0;
    public static final double setIntakeSpeed = 1.0;
    public static final double setFoldSpeed = 0.09;
    
  }
   public static class reefConstants {
    public static final double reefX = 4.491799831390381;
    public static final double reefY = 4.026460647583008;
    public static final double[] pointsX = { 5.8339008113657025 , 5.668800811365703 , 4.326700000000001 , 3.1496991886342975 , 3.3147991886342965 , 4.6568999999999985 , 5.668800811365703 , 5.8339008113657025 , 4.6569 , 3.3147991886342973 , 3.149699188634296 , 4.326699999999998};
    public static final double[] pointsY = {4.61068120583519 , 3.15627720583519 , 2.572056 , 3.44223879416481 , 4.89664279416481 , 5.480864 , 4.896642794164811 , 3.442238794164811 , 2.572056 , 3.1562772058351882 , 4.6106812058351885 , 5.480864};

    }

    public static class driveConstants {
      public static class lowerTrench {
        public static final double x1 = 1.0;
        public static final double y1 = 4.0;
        public static final double x2 = 1.0;
        public static final double y2 = 4.0;
        public static final double x3 = 1.0;
        public static final double y3 = 4.0;
      }
        public static class upperTrench {
          public static final double x1 = 1.0;
          public static final double y1 = 4.0;
          public static final double x2 = 1.0;
          public static final double y2 = 4.0;
          public static final double x3 = 1.0;
          public static final double y3 = 4.0;
        }

    }
}
