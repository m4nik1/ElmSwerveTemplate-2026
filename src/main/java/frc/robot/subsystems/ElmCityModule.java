package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElmCityModule extends SubsystemBase {
  /** Creates a new ElmCityKrakenModule. */

  TalonFX driveMotor;
  TalonFX angleMotor;

  AnalogEncoder nacCoder;


  PositionDutyCycle anglePosition = new PositionDutyCycle(0);
  DutyCycleOut driveOpenLoop;
  VelocityVoltage driveVelocity;
  SimpleMotorFeedforward driveKfCalc = new SimpleMotorFeedforward(Constants.driveKs, Constants.driveKv, Constants.drivekA);

  public int modNum;
  public double velocitySet;

  Rotation2d lastAngle;
  Rotation2d offset;
  InvertedValue driveInverted;
  int resetTimeout_MS;

  public ElmCityModule(int moduleNumber, int driveID, int angleID, int nacID, Rotation2d givenOffset,
      InvertedValue driveInvert,
      InvertedValue angleInvert) {
    this.modNum = moduleNumber;
    this.offset = givenOffset;
    this.driveInverted = driveInvert;

    driveMotor = new TalonFX(driveID);
    angleMotor = new TalonFX(angleID);

    nacCoder = new AnalogEncoder(nacID);

    velocitySet = 0;
    resetTimeout_MS = 250;

    // 0 is default for PID constants
    anglePosition.Slot = 0;

    driveOpenLoop = new DutyCycleOut(0);
    driveVelocity = new VelocityVoltage(0);
    driveVelocity.Slot = 0;

    configDriveMotor(driveInvert);
    configAngleMotor(angleInvert);

    driveMotor.setPosition(0.0, resetTimeout_MS);
    lastAngle = Rotation2d.fromDegrees(0);
    resetToAbsolute();
    // setAngleZero();
  }

  public void configDriveMotor(InvertedValue drive) {
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();

    driveConfig.MotorOutput.Inverted = drive;
    driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    driveConfig.Feedback.SensorToMechanismRatio = Constants.driveRatio; ///((Math.PI)*Units.inchesToMeters(4))
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.CurrentLimits.StatorCurrentLimit = 60;

    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.SupplyCurrentLimit = 60;


    driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.15;
    driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.15;
    driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.15;

    driveConfig.Slot0.kP = Constants.driveKp;
    driveConfig.Slot0.kI = Constants.driveKi;
    driveConfig.Slot0.kD = Constants.driveKd;

    driveMotor.getConfigurator().apply(driveConfig);
    driveMotor.getConfigurator().setPosition(0.0, resetTimeout_MS);


  }

  public void configAngleMotor(InvertedValue invertedValue) {
    TalonFXConfiguration angleConfig = new TalonFXConfiguration();

    angleConfig.MotorOutput.Inverted = invertedValue;
    angleConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    angleConfig.Feedback.SensorToMechanismRatio = Constants.angleRatio;
    angleConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    angleConfig.ClosedLoopGeneral.ContinuousWrap = true;

    angleConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    angleConfig.CurrentLimits.StatorCurrentLimit = 60;

    angleConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
    angleConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    angleConfig.Slot0.kP = Constants.angleP;
    angleConfig.Slot0.kI = Constants.angleI;
    angleConfig.Slot0.kD = Constants.angleD;

    angleMotor.getConfigurator().apply(angleConfig);
  }

  public void resetToAbsolute() {
    double absoluteValue = getNac() - offset.getRotations();
    angleMotor.setPosition(absoluteValue);
  }

  public void setAngleZero() {
    angleMotor.setPosition(0);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosMeters(), Rotation2d.fromRotations(getAngle()));
  }

  public double getNac() {
    return nacCoder.get();
  }

  public double getDrivePosition() {
    return driveMotor.getPosition().getValueAsDouble();
  }

  public double getDriveVel() {
    return driveMotor.getVelocity().getValueAsDouble();
  }

  public double getAngle() {
    return angleMotor.getPosition().getValueAsDouble();
  }

  public double getAngleDegrees() {
    return angleMotor.getPosition().getValueAsDouble() * 360;
  }


  public void setSpeed(SwerveModuleState desiredState, boolean openLoop) {
    if (openLoop) {
      driveOpenLoop.Output = desiredState.speedMetersPerSecond / Constants.maxSpeed;
      driveMotor.setControl(driveOpenLoop); // change this to a set speed if its not smooth
    }
    
    // add closed loop for driveController
    else {
      // converts velocity to Rotation per second with wheel curcumfirence
      driveVelocity.Velocity = desiredState.speedMetersPerSecond / Constants.wheelCircum;

      driveVelocity.Slot = 0;

      // sets the feedforward to simple feedforward calculation with the requested speed
      driveVelocity.FeedForward = driveKfCalc.calculate(desiredState.speedMetersPerSecond);

      // Calculate using feedforward
      driveMotor.setControl(driveVelocity);
    }
  }

  public void setAngle(SwerveModuleState desiredState) {
    Rotation2d angle;
    double absSpd = Math.abs(desiredState.speedMetersPerSecond);
    anglePosition.Slot = 0;

    if (absSpd <= (Constants.maxSpeed * .01)) {
      angle = lastAngle;
    } else {
      angle = desiredState.angle;
    }

    anglePosition.Position = angle.getRotations();
    angleMotor.setControl(anglePosition);
    lastAngle = angle;

  }

  public void runVelocity(double vel) {
    driveVelocity.Velocity = vel / Constants.wheelCircum; // meters per second to rotations per second

    velocitySet = vel;


    driveVelocity.Slot = 0;

    // sets the feedforward to simple feedforward calculation with the requested
    driveVelocity.FeedForward = driveKfCalc.calculate(vel);

    // Calculate using feedforward
    driveMotor.setControl(driveVelocity);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean openLoop) {
    SwerveModuleState newState = new SwerveModuleState();

    Rotation2d currentAngle = Rotation2d.fromRotations(getAngle());

    newState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    newState.angle = desiredState.angle;

    newState.optimize(Rotation2d.fromRotations(getAngle()));

    // Test this out for smoother driving
    newState.speedMetersPerSecond *= newState.angle.minus(currentAngle).getCos();
    
    setSpeed(newState, openLoop);
    setAngle(newState);
  }

  public void goToAngle(double deg) {
    anglePosition.withSlot(0);
    anglePosition.Position = Rotation2d.fromDegrees(deg).getRotations();

    angleMotor.setControl(anglePosition);
  }

  public double getDriveVelocityConversion() {
    double meters = driveMotor.getVelocity().getValueAsDouble() * Constants.wheelCircum;
    // SmartDashboard.putString("Units of velocity", driveMotor.getVelocity().getUnits());
    
    return meters;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveMotor.getVelocity().getValueAsDouble(), Rotation2d.fromRotations(getAngle()));
  }

  public double getDrivePosMeters() {
    double velocity = driveMotor.getPosition().getValueAsDouble() * Constants.wheelCircum;

    return velocity;
  }

  public void setVel(double vel) {
    // converts velocity to Rotation per second with wheel curcumfirence
    driveVelocity.Velocity = vel / Constants.wheelCircum;

     driveVelocity.Slot = 0;
    
    // sets the feedforward to simple feedforward calculation with the requested speed
    driveVelocity.FeedForward = driveKfCalc.calculate(vel);
    
    // Calculate using feedforward
    driveMotor.setControl(driveVelocity);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Distance (M)" + modNum, getDrivePosMeters());
    SmartDashboard.putNumber("Nac coder rot " + modNum, getNac());
    SmartDashboard.putNumber("Module Angle "+ modNum, getAngleDegrees());

    Logger.recordOutput("Distance (M)" + modNum, getDrivePosMeters());
    Logger.recordOutput("Velocity " + modNum , getDriveVelocityConversion());
    Logger.recordOutput("Module Angle " + modNum, getAngleDegrees());
  }
}
