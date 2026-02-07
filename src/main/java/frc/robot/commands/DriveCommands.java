package frc.robot.commands;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class DriveCommands {

    static PIDController rotationController = new PIDController(.5, 0, 0);

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

            Logger.recordOutput("vision angle diff", RobotContainer.visionLeft.getYawAlign()-RobotContainer.driveTrain.getRobotAngle());

            // Calculate and apply deadband the values of each
            double translateVal = translationLimiter.calculate(speedMultipler*MathUtil.applyDeadband(getX,.01));
            double strafeVal = strafeLimiter.calculate(speedMultipler*MathUtil.applyDeadband(getY,.01));
            double rotationVal = rotationLimiter.calculate(speedMultipler*MathUtil.applyDeadband(getRotation,.01));

            // Add the translate and strafe values to translate2d object
            Translation2d translation = new Translation2d(translateVal, strafeVal);

            // If A is being held down, auto aim while moving, else normal drive 
            if(RobotContainer.getA()){
                autoAimMove(xSupplier, ySupplier);
            }
            else {
                // Send the translation and rotation values to drive object
                RobotContainer.driveTrain.drive(translation.times(Constants.maxSpeed),rotationVal*Constants.maxAngularSpd);
            }
        }, RobotContainer.driveTrain);
    }

    // Auto aim command
    // We want the driver to move while the robot is angled towards the tags
    public static void autoAimMove(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        // Slew rate limit
        SlewRateLimiter translationLimiter = new SlewRateLimiter(3);
        SlewRateLimiter strafeLimiter = new SlewRateLimiter(3);
        SlewRateLimiter rotationLimiter = new SlewRateLimiter(3);

        // Call the vision's getYawAlign
        double angle = RobotContainer.visionLeft.getYawAlign()-RobotContainer.driveTrain.getRobotAngle();

        double getX = xSupplier.getAsDouble();
        double getY = ySupplier.getAsDouble();

        // Add translation and strafe values
        double translate = translationLimiter.calculate(.8*MathUtil.applyDeadband(getX, .01));
        double strafe = strafeLimiter.calculate(.8*MathUtil.applyDeadband(getY, .01));
        double rotation = rotationController.calculate(angle, 0);

        // Make translation object
        Translation2d translation = new Translation2d(translate, strafe);

        // Send the translation values to drive
        RobotContainer.driveTrain.drive(translation.times(Constants.maxSpeed),rotation*5);
    }
}
