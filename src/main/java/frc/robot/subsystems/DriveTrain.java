
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
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
  final SysIdRoutine m_sysIdRoutine;

  public DriveTrain() {

    // Make the module array
    elmCityModules = new ElmCityModule[] {
      new ElmCityModule(0, 8, 7, 0,Constants.angleOffsetMod0,InvertedValue.CounterClockwise_Positive, InvertedValue.Clockwise_Positive),
      new ElmCityModule(1, 20, 19, 2, Constants.angleOffsetMod1, InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive),
      new ElmCityModule(2, 10, 9, 1, Constants.angleOffsetMod2 ,InvertedValue.CounterClockwise_Positive, InvertedValue.Clockwise_Positive),
      new ElmCityModule(3, 17, 18, 3, Constants.angleOffsetMod3, InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive),
    };

    // Make the gyro
    gyro = new Pigeon2(Constants.pigeonID);

    // Make odom variable
    odom = new SwerveDrivePoseEstimator(Constants.swerveKinematics, 
                                    getYaw(), 
                                    getPositions(), 
                                    new Pose2d(), 
                                    VecBuilder.fill(0.01, 0.01, 0.01), // Try (0.005, 0.005, 0.005) to trust odom more
                                    VecBuilder.fill(0.04, 0.04, 0.06)); // Try (0.05, .05, .08) to trust vision a little less

    // Call Reset gyro at startup
    resetGyro();

    final VoltageOut m_sysIdControl = new VoltageOut(0);
    this.setAnglesZero();

    m_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,         // Default ramp (1V/s)
            Volts.of(4),  // Max step voltage
            null,         // Default timeout
            (state) -> SignalLogger.writeString("state", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (volts) -> {
                // 2. Apply voltage to ALL 4 drive motors
                // Also: Ensure your steer motors are pointing the wheels at 0 degrees!
                elmCityModules[0].getDriveMotor().setControl(m_sysIdControl.withOutput(volts));
                elmCityModules[1].getDriveMotor().setControl(m_sysIdControl.withOutput(volts));
                elmCityModules[2].getDriveMotor().setControl(m_sysIdControl.withOutput(volts));
                elmCityModules[3].getDriveMotor().setControl(m_sysIdControl.withOutput(volts));
                
                // Explicitly tell steer motors to stay at 0 during the test
                // (Assumes you have a method to set steer position)
                this.setAnglesZero(); 
            },
            null, // SignalLogger automatically records the motor data
            this
        )
    );

    // AutoBuilder goes here for auto

  }

  // Return the angle of robot in Rotation2d
  public Rotation2d getYaw() {
    return gyro.getRotation2d();
  }

  public Pose2d getPose() {
    return odom.getEstimatedPosition();
  }

  public SwerveModulePosition[] getPositions() {

    // 1. Make the positions array variable
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    // 2. Get each module position using for loop
    for (ElmCityModule m : elmCityModules) {
      positions[m.modNum] = m.getPosition();
    }

    // Return the positions
    return positions;
  }

  // test command for velocity tuning
  public Command runVelocityCommand() {
    return run(() -> elmCityModules[0].runVelocity(0.5));
  }

  // Resets gyro for robot
  public void resetGyro() {
    gyro.reset();
  }

  public void drive(Translation2d translation, double rotation) {
    SwerveModuleState[] moduleStates;

    // 1. Convert to field relative speeds
    ChassisSpeeds spds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw());

    // 2. discretize the speeds to make it accurate using .discretize
    spds = ChassisSpeeds.discretize(spds, .02);

    // 3. Now convert the speeds to each moduleState based on location
    moduleStates = Constants.swerveKinematics.toSwerveModuleStates(spds);

    // 4. Desaturate wheel speeds to ensure each wheel is at Constants.maxSpeed
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.maxSpeed);

    // 5. set the desired state using for loop
    for(ElmCityModule m : elmCityModules) {
      m.setDesiredState(moduleStates[m.modNum], true);
    }
  }

  public void setAnglesZero() {
    for(ElmCityModule m : elmCityModules) {
      m.setAngleZero();
    }
  }

  public Command setAngleZero() {
    return run(() -> {
      for(ElmCityModule m : elmCityModules) {
        m.setAngleZero();
      }
    });
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }
 
 public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public void updatePoseEstimate(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
    odom.addVisionMeasurement(robotPose, timestamp, stdDevs);
  }

  public double getRobotAngle() {
    return gyro.getYaw().getValueAsDouble();
  }

  // Tuning the angle PID
  public void setAngle(double deg) {

  }

  // resets all the wheel angles
  public void zeroAngles() {
  }

  @Override
  public void periodic() {
    // Update pose with odometry using odom
    odom.update(getYaw(), getPositions());

    Logger.recordOutput("Robot Pose", odom.getEstimatedPosition());
    Logger.recordOutput("Robot Angle", getRobotAngle());
  }
}