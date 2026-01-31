package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class DriveCommands {

    // Drive only in teleop
    public static Command teleopDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier) {
        double speedMultipler = Constants.speedMultiTeleop;
        SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
        SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
        SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

        return Commands.run(() ->  {
            double getX = xSupplier.getAsDouble();
            double getY = ySupplier.getAsDouble();
            double getRotation = rotationSupplier.getAsDouble();

            // Use a small deadband (0.05) so small joystick inputs still produce motion.
            double translateVal = translationLimiter.calculate(speedMultipler * MathUtil.applyDeadband(getX, 0.05));
            double strafeVal = strafeLimiter.calculate(speedMultipler * MathUtil.applyDeadband(getY, 0.05));
            double rotationVal = rotationLimiter.calculate(speedMultipler * MathUtil.applyDeadband(getRotation, 0.05));

            Translation2d translation = new Translation2d(translateVal, strafeVal);

            RobotContainer.driveTrain.drive(translation.times(Constants.maxSpeed), rotationVal * Constants.maxAngularSpd);

        }
        , RobotContainer.driveTrain);
    }
}
