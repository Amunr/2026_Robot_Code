package frc.robot.subsystems;

import java.io.File;
import java.nio.file.Path;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Meter;
public class DriveTrain extends SubsystemBase {
  public SwerveDrive swerveDrive;
  public double maximumSpeed = Units.feetToMeters(20);
  public Vision visionSubsystem;
  private boolean k_vision;
  public Command dynamicPath;
  public DriveTrain(File directory) {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    try {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
          new Pose2d(new Translation2d(Meter.of(1),
              Meter.of(4)),
              Rotation2d.fromDegrees(0)));
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false);
    swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
    swerveDrive.setAngularVelocityCompensation(true,
        true,
        0.1);
    swerveDrive.setModuleEncoderAutoSynchronize(false,
        1);


      setupPhotonVision();
  

  }

  /**
   * Command to drive the robot using translative values and heading as a
   * setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  // public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
  //     DoubleSupplier angularRotationX) {
  //   return run(() -> {
  //     // Make the robot move
  //     swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
  //         translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
  //         translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
  //         Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
  //         isFieldRelative(),
  //         false);
  //   });
  // }

//   private boolean isFieldRelative() {
//     var alliance = DriverStation.getAlliance();
//     return !(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red);
// }
/* 
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(translation,
        rotation,
        fieldRelative,
        false); // Open loop is disabled since it shouldn't be used most of the time.
  }
*/
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  // PATHPLANNAR
  public void setupPathPlanner() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder lastwxsd66e55555555556666
      AutoBuilder.configure(
          this::getPose,
          // Robot pose supplier
          this::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
          // optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic
              // drive trains
              new PIDConstants(1, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(1, 0.0, 0.0)
          // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
      // Reference to this subsystem to set requirements
      );

    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

  // Path pallannar functions
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  // drivetoSetpoint
  public void setupPhotonVision() {
    k_vision = true;
    visionSubsystem = new Vision();
  }

  public double setAutoDouble() {
    return 0;
  }

  @Override
  public void periodic() {
    swerveDrive.updateOdometry();


      visionSubsystem.updatePoseEstimation(swerveDrive, swerveDrive.getPose());
      SmartDashboard.putNumber("Pose X", getPose().getX());
      SmartDashboard.putNumber("PoseY", getPose().getY());
      SmartDashboard.putNumber("Pose rotation", getPose().getRotation().getDegrees());

  }

  public Pose2d nearestReef(Pose2d robotPos){
        var c = 8.774176*2;
        var dist = 500;
       //NOTE DOES DIST NEED TO BE CHANCGED TO THE CLOOSEST?
        var centX = Constants.reefConstants.reefX;
        var centY = Constants.reefConstants.reefY;
        double xd=0;
        double yd=0;
        var alliance = DriverStation.getAlliance();
        for(var i=0;i<Constants.reefConstants.pointsX.length;i++){
            var xpos = Constants.reefConstants.pointsX[i];
            var ypos = Constants.reefConstants.pointsY[i];
            if(alliance.get() == DriverStation.Alliance.Red){
                xpos= xpos * (-1) + c;
                //ypos= ypos * (-1) + c;
                centX =  Constants.reefConstants.reefX* (-1) + c;
            }
            if( (robotPos.getX()-xpos)*(robotPos.getX()-xpos) + (robotPos.getY()-ypos)*(robotPos.getY()-ypos)  < dist){
               
                xd=xpos;
                yd=ypos;
            }
        }
       //BRET DOES THIS RETURN THE FINIAL POSTION OR THE TRANSLATION 2D to get there. We just need the closest final position. 
        return new Pose2d(new Translation2d(xd,yd), new Rotation2d(xd-centX,yd-centY));


        //2d translation excepts a anlge in radians not vectors? 
    }

 
    

public void getToPoint(){
  List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        getPose(),
        nearestReef(getPose())); 

PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.

// Create the path using the waypoints created above
PathPlannerPath path = new PathPlannerPath(
        waypoints,
        constraints,
        null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
        new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
);
  dynamicPath = AutoBuilder.followPath(path);
  dynamicPath.schedule();
}

public void cancelGetToPoint(){
  if (dynamicPath != null && dynamicPath.isScheduled()) {
  dynamicPath.cancel();
  }
}
}