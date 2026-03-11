// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.autoCommands.LeftSidePeak;
import frc.robot.autoCommands.RightSidePeak;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Vision;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.IdentityHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private static final String AUTO_CHOOSER_KEY = "autoChooser";

  // The robot's subsystems and commands are defined
  public static DriveTrain driveTrain = new DriveTrain();
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final Map<Command, String> commandToAutoFile = new IdentityHashMap<>();

  // public static Vision visionRight = new Vision("elm_back_cam", new
  // Transform3d(
  // Units.inchesToMeters(-3.5), Units.inchesToMeters(13), 0.0,
  // new Rotation3d(0, Units.degreesToRadians(-20), 0)
  // ), true);

  public static Vision visionLeft = new Vision(
      "elm_front_cam", new Transform3d(
          Units.inchesToMeters(-3.5), Units.inchesToMeters(-13), 0.0,
          new Rotation3d(0, Rotation2d.fromDegrees(-20).getRadians(), 0)),
      false);

  public static CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    // Configure the trigger bindings
    driveTrain.setDefaultCommand(DriveCommands.teleopDrive(
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> -controller.getRightX()));

    configureBindings();
    // Add new autos here. Display name is what appears in Elastic/SmartDashboard
    // chooser,
    // auto file name must match src/main/deploy/pathplanner/autos/<name>.auto
    // (without .auto).
    registerAuto("StraightAutotest", "Straight test P", true);
    registerAuto("Right Side trench", "Right Side 90", false);
    registerAuto("Center Auto", "Center Auto", false);
    try {
     autoChooser.addOption("Left Side trench peak", new LeftSidePeak().getAuto());
     autoChooser.addOption("Right Side trench peak", new RightSidePeak().getAuto());
    } catch (IOException | ParseException e) {
      e.printStackTrace();
    }

    SmartDashboard.putData(AUTO_CHOOSER_KEY, autoChooser);
  }

  private void configureBindings() {
    // controller.a().onTrue(DriveCommands.autoAimMove(()->-controller.getLeftY(),()
    // ->-controller.getRightX()));
    controller.leftBumper().onTrue(driveTrain.zeroGyro());
  }

  public static boolean getA() {
    return controller.a().getAsBoolean();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void registerAuto(String displayName, String autoFileName, boolean isDefault) {
    // Example for a new auto:
    // registerAuto("Center2Piece", "Center 2 Piece", false);
    // Chooser autos come from PathPlanner .auto/.path files, so editing
    // testFollowPath.java
    // alone will not change what this selected autonomous routine does.
    Command autoCommand = new PathPlannerAuto(autoFileName);
    commandToAutoFile.put(autoCommand, autoFileName);
    if (isDefault) {
      autoChooser.setDefaultOption(displayName, autoCommand);
    } else {
      autoChooser.addOption(displayName, autoCommand);
    }
  }
}
