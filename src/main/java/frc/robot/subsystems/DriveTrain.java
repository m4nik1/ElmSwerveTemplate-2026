
package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new TestDrive. */

  ElmCityModule[] elmCityModules;

  Pigeon2 gyro;
  Pose2d robotPose;

  // Make the SwerveDrive Estimator
  SwerveDrivePoseEstimator odom;

  // Path planner config
  RobotConfig autoConfig;

  public DriveTrain() {

    // Make the module array
    elmCityModules = new ElmCityModule[]{
      new ElmCityModule(0, 8, 7, 0, Constants.angleOffsetMod0, InvertedValue.CounterClockwise_Positive, InvertedValue.Clockwise_Positive),
      new ElmCityModule(1, 20, 19,2,Constants.angleOffsetMod1,InvertedValue.Clockwise_Positive,InvertedValue.Clockwise_Positive ),
      new ElmCityModule(2, 10, 9,1,Constants.angleOffsetMod2 ,InvertedValue.CounterClockwise_Positive,InvertedValue.Clockwise_Positive ),
      new ElmCityModule(3, 17, 18, 3,Constants.angleOffsetMod3,InvertedValue.Clockwise_Positive,InvertedValue.Clockwise_Positive),
    };
    // Make the gyro (pigeon)
    gyro = new Pigeon2(21);
    

    // Make odom variable:
    odom = new SwerveDrivePoseEstimator(Constants.swerveKinematics,
                                        getYaw(), 
                                        getPositions(), 
                                        new Pose2d(), 
                                        VecBuilder.fill(0.1, 0.1, 0.1), 
                                        VecBuilder.fill(0.9, 0.9, 0.9)
                                      );

    try {
      autoConfig = RobotConfig.fromGUISettings();
    } catch(Exception e) {
      e.printStackTrace();
    }

    // Call Reset gyro at startup
    gyro.reset();


    // AutoBuilder goes here for auto
    AutoBuilder.configure(
      this::getPose, 
      this::resetPose, 
      this::getRobotSpds, 
      (speeds, feedforwards) -> driveRobotRelative(speeds), 
      new PPHolonomicDriveController(
          new PIDConstants(4.7, 0, 0), // XY PID
          new PIDConstants(4.5, 0, 0)  // Rotational PID
        ), 
      autoConfig, 
      () -> {
        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      }, 
      this
    );
  }

  // Return the angle of robot in Rotation2d
  public Rotation2d getYaw() {
    return gyro.getRotation2d();
  }

  public Pose2d getPose() {
    return odom.getEstimatedPosition();
  }

  public ChassisSpeeds getRobotSpds() {
    return Constants.swerveKinematics.toChassisSpeeds(
        elmCityModules[0].getState(),
        elmCityModules[1].getState(),
        elmCityModules[2].getState(),
        elmCityModules[3].getState());
  }

  public void driveRobotRelative(ChassisSpeeds spds) {
    SwerveModuleState states[];
    ChassisSpeeds spds_discrete = ChassisSpeeds.discretize(spds, .02);
    states = Constants.swerveKinematics.toSwerveModuleStates(spds_discrete);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.maxSpeed);

    for (ElmCityModule m : elmCityModules) {
      m.setDesiredState(states[m.modNum], false);
    }
  }

  public void resetPose(Pose2d pose) {
    odom.resetPosition(getYaw(), getPositions(), pose);
  }

  public SwerveModulePosition[] getPositions() {

    // 1. Make the positions array variable
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    // 2. Get each module position using for loop
    for (ElmCityModule mod : elmCityModules) {
      positions[mod.modNum] = mod.getPosition();
    }
    // Return the positions
    return positions;
  }

  public void drive(Translation2d translation, double rotation) {
    SwerveModuleState[] moduleStates;

    // 1. Convert to field relative speeds
    ChassisSpeeds spds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
        getYaw());

    // 2. discretize the speeds to make it accurate using .discretize
    spds = ChassisSpeeds.discretize(spds, .02);
    // 3. Now convert the speeds to each moduleState based on location
    moduleStates = Constants.swerveKinematics.toSwerveModuleStates(spds);
    // 4. Desaturate wheel speeds to ensure each wheel is at Constants.maxSpeed
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.maxSpeed);
    // 5. set the desired state using for loop
    for (ElmCityModule mod : elmCityModules) {
      mod.setDesiredState(moduleStates[mod.modNum], true);
    }
  }

  public Pose2d getRobotPose2d() {
    return odom.getEstimatedPosition();
  }

  public void updatePoseEstimate(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
    // System.out.println("Std devs: " + stdDevs);
    odom.addVisionMeasurement(new Pose2d(pose.getX(), pose.getY(), getYaw()), timestamp, stdDevs);
  }

  public double getRobotAngle() {
    return gyro.getYaw().getValueAsDouble();
  }

  // Tuning the angle PID
  public void setAngle(double deg) {
    elmCityModules[0].goToAngle(deg);
  }

  public Command zeroGyro() {
    return run(() -> {
      gyro.setYaw(0);
    });
  }

  public Command setAngleCommand() {
    return run(() -> {
      for (ElmCityModule mod : elmCityModules) {
        elmCityModules[mod.modNum].goToAngle(90);
      }
    });
  }

  @Override
  public void periodic() {
    odom.update(getYaw(), getPositions());
    // Update pose with odometry using odom
    Logger.recordOutput("robotAngle", getYaw().getDegrees());
    Logger.recordOutput("robotPose", getPose());

  }
}