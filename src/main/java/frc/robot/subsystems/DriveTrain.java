
package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.InvertedValue;

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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new TestDrive. */

  ElmCityModule[] elmCityModules;

  Pigeon2 gyro;
  Pose2d robotPose;

  // Make the SwerveDrive Estimator
  SwerveDrivePoseEstimator odom;


  public DriveTrain() {

    // Make the module array
    elmCityModules = 

    // Make the gyro (pigeon)
    gyro = 

    // Make odom variable
    odom =

    // Call Reset gyro at startup


    // AutoBuilder goes here for auto

  }

  // Return the angle of robot in Rotation2d
  public Rotation2d getYaw() {
  }

  public Pose2d getPose() {
  }

  public SwerveModulePosition[] getPositions() {

    // 1. Make the positions array variable
    SwerveModulePosition[] positions = 

    // 2. Get each module position using for loop

    // Return the positions
    return positions;
  }

  // test command for velocity tuning
  public Command runVelocityCommand() {
  }

  // Resets gyro for robot
  public void resetGyro() {
  }

  public void drive(Translation2d translation, double rotation) {
    SwerveModuleState[] moduleStates;

    // 1. Convert to field relative speeds
    ChassisSpeeds spds = 

    // 2. discretize the speeds to make it accurate using .discretize

    // 3. Now convert the speeds to each moduleState based on location

    // 4. Desaturate wheel speeds to ensure each wheel is at Constants.maxSpeed

    // 5. set the desired state using for loop
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
}

  public Pose2d getRobotPose2d() {
  }


  public double getRobotAngle() {
  }

  // Tuning the angle PID
  public void setAngle(double deg){

  }

  // resets all the wheel angles
  public void zeroAngles() {
  }

  @Override
  public void periodic() {
    // Update pose with odometry using odom

  }
}