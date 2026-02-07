// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  // The robot's subsystems and commands are defined here
  public static DriveTrain driveTrain = new DriveTrain();
  
  public static Vision visionRight = new Vision("elm_right_cam", new Transform3d(
    Units.inchesToMeters(9.25), Units.inchesToMeters(13), 0.0, 
    new Rotation3d(0, Rotation2d.fromDegrees(-20).getRadians(), 0)
  ));

  public static Vision visionLeft = new Vision("elm_left_cam", new Transform3d(
    Units.inchesToMeters(9.25), Units.inchesToMeters(-13), 0.0, 
    new Rotation3d(0, Rotation2d.fromDegrees(-20).getRadians(), 0)
  ));

  public static CommandXboxController controller= new CommandXboxController(0);

  public RobotContainer() {
    // Configure the trigger bindings
    driveTrain.setDefaultCommand(DriveCommands.teleopDrive(
      ()->-controller.getLeftY(), 
      ()->-controller.getLeftX() , 
      ()->-controller.getRightX()
    ));

    configureBindings();
  }

  private void configureBindings() {
    // controller.a().onTrue(DriveCommands.autoAimMove(()->-controller.getLeftY(),() ->-controller.getRightX()));
    controller.leftBumper().onTrue(driveTrain.zeroGyro());
  }

  public static boolean getA(){
    return controller.a().getAsBoolean();
  }

  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   // return Autos.exampleAuto();
  // }
}
