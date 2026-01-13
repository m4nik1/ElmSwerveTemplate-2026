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
        // Setup variables

        // Speed multipler for driving
        double speedMultipler = Constants.speedMultiTeleop;

        // Slew Rate Limiters for smooth driving
        SlewRateLimiter translationLimiter = 
        SlewRateLimiter strafeLimiter = 
        SlewRateLimiter rotationLimiter = 

        // Now running the command
        return Commands.run(() ->  {
            // Get the joystick inputs
            double getX = 
            double getY = 
            double getRotation = 

            // Calculate and apply deadband the values of each
            double translateVal = 
            double strafeVal = 
            double rotationVal = 

            // Add the translate and strafe values to translate2d object
            Translation2d translation = 

            // Send the translation and rotation values to drive object
            RobotContainer.driveTrain.drive();

        }
        , RobotContainer.driveTrain);
    }
}
