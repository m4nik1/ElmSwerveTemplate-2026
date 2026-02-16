package frc.robot.commands;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class DriveCommands {

    static SlewRateLimiter autoAimTranslationLimiter = new SlewRateLimiter(3);
    static SlewRateLimiter autoAimStrafeLimiter = new SlewRateLimiter(3);
    static SlewRateLimiter autoAimRotationLimiter = new SlewRateLimiter(3);
    static PIDController rotationController = new PIDController(.02, 0, 0); // old 35

    static {
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    // Drive only in teleop
    public static Command teleopDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier) {
        // Setup variables

        // Speed multipler for driving
        double speedMultipler = Constants.speedMultiTeleop;

        // Slew Rate Limiters for smooth driving
        SlewRateLimiter translationLimiter = new SlewRateLimiter(3);
        SlewRateLimiter strafeLimiter = new SlewRateLimiter(3);
        SlewRateLimiter rotationLimiter = new SlewRateLimiter(3);

        // Now running the command
        return Commands.run(() ->  {
            // Get the joystick inputs
            double getX = xSupplier.getAsDouble();
            double getY = ySupplier.getAsDouble();
            double getRotation = rotationSupplier.getAsDouble();

            // Logger.recordOutput("vision angle diff", RobotContainer.visionLeft.getYawAlign()-RobotContainer.driveTrain.getRobotAngle());

            // Calculate and apply deadband the values of each
            double translateVal = translationLimiter.calculate(speedMultipler*MathUtil.applyDeadband(getX,.01));
            double strafeVal = strafeLimiter.calculate(speedMultipler*MathUtil.applyDeadband(getY,.01));
            double rotationVal = rotationLimiter.calculate(speedMultipler*MathUtil.applyDeadband(getRotation, .06));

            // Add the translate and strafe values to translate2d object
            Translation2d translation = new Translation2d(translateVal, strafeVal);

            // If A is being held down, auto aim while moving, else normal drive 
            if(RobotContainer.getA() && RobotContainer.visionRight.targetFound()){
                simpleAutoAimMove(xSupplier, ySupplier);
            }
            else {
                // Send the translation and rotation values to drive object
                RobotContainer.driveTrain.drive(translation.times(Constants.maxSpeed),rotationVal*Constants.maxAngularSpd);
            }
        }, RobotContainer.driveTrain);
    }

    // Auto aim command
    // We want the driver to move while the robot is angled towards the tags
    public static void simpleAutoAimMove(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        // Call the vision's getYawAlign
        double targetYaw = RobotContainer.visionRight.getYawAlign();

        double getX = xSupplier.getAsDouble();
        double getY = ySupplier.getAsDouble();

        // Add translation and strafe values
        double translate = autoAimTranslationLimiter.calculate(.8 * MathUtil.applyDeadband(getX, .01));
        double strafe = autoAimStrafeLimiter.calculate(.8 * MathUtil.applyDeadband(getY, .01));

        Logger.recordOutput("Get camera yaw", targetYaw);  
        System.out.println("yaw " + targetYaw);

        // Make the PID loop calculate
        double rotation = rotationController.calculate(targetYaw, 0);
        
        // Make translation object
        Translation2d translation = new Translation2d(translate, strafe);

        Logger.recordOutput("Rotation output", rotation);

        // Send the translation values to drive
        RobotContainer.driveTrain.drive(translation.times(Constants.maxSpeed), rotation * (Math.PI * 2));
    }

    // Auto aim command
    // We want the driver to move while the robot is angled towards the tags
    public static void autoAimOffset(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        // Call the vision's getYawAlign
        double targetYaw = RobotContainer.visionRight.getYawAlign();
        double alignDistance = RobotContainer.visionRight.getAlignDistance();

        double cameraOffset = 0.3302;
        if(alignDistance > 0.1) {
            double offsetCorrectionDeg = Math.toDegrees(Math.atan(cameraOffset / alignDistance));
            targetYaw -= offsetCorrectionDeg;
        }

        double getX = xSupplier.getAsDouble();
        double getY = ySupplier.getAsDouble();

        // Add translation and strafe values
        double translate = autoAimTranslationLimiter.calculate(.8 * MathUtil.applyDeadband(getX, .01));
        double strafe = autoAimStrafeLimiter.calculate(.8 * MathUtil.applyDeadband(getY, .01));

        Logger.recordOutput("Get camera yaw", targetYaw);  
        System.out.println("yaw " + targetYaw);

        // Make the PID loop calculate
        double rotation = rotationController.calculate(targetYaw, 0);
        
        // Make translation object
        Translation2d translation = new Translation2d(translate, strafe);

        Logger.recordOutput("Rotation output", rotation);

        // Send the translation values to drive
        RobotContainer.driveTrain.drive(translation.times(Constants.maxSpeed), rotation * (Math.PI * 2));
    }

    public static void autoAimMovePoses(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        double getX = xSupplier.getAsDouble();
        double getY = ySupplier.getAsDouble();

        // Add translation and strafe values
        double translate = autoAimTranslationLimiter.calculate(.8 * MathUtil.applyDeadband(getX, .01));
        double strafe = autoAimStrafeLimiter.calculate(.8 * MathUtil.applyDeadband(getY, .01));

        Pose3d tag3 = Constants.fieldLayout.getTagPose(3).orElseThrow();
        Pose3d tag4 = Constants.fieldLayout.getTagPose(4).orElseThrow();

        Translation2d hubOffset = tag3.toPose2d().getTranslation().plus(tag4.toPose2d().getTranslation()).div(2);

        Pose2d robotPose = RobotContainer.driveTrain.getPose();

        double diffX = hubOffset.getX() - robotPose.getX();
        double diffY = hubOffset.getY() - robotPose.getY();

        double desiredAngle = Math.atan2(diffY, diffX);
        double currentAngle = robotPose.getRotation().getRadians();

        double rotation = rotationController.calculate(currentAngle, desiredAngle);
        
        // Make translation object
        Translation2d translation = new Translation2d(translate, strafe);

        Logger.recordOutput("Rotation output", rotation);

        // Send the translation values to drive
        RobotContainer.driveTrain.drive(translation.times(Constants.maxSpeed), rotation);
    }
}
