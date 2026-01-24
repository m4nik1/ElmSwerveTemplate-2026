// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  // The robot's subsystems and commands are defined here
  public static DriveTrain driveTrain = new DriveTrain();
  public CommandXboxController controller= new CommandXboxController(0);

  public RobotContainer() {
    // Configure the trigger bindings
    // drivetrain.setDefaultCommand(DriveCommands.teleopDrive(null, null, null))

    configureBindings();
  }

  private void configureBindings() {
    controller.a().onTrue(driveTrain.setAngleCommand());
  }

  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   // return Autos.exampleAuto();
  // }
}
