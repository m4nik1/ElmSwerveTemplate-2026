// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  private static final String AUTO_CHOOSER_KEY = "autoChooser";
  private static final String AUTO_PREVIEW_KEY = "Auto Preview";
  private static final String AUTO_PREVIEW_OBJECT_KEY = "Selected Auto Path";
  private static final String SELECTED_AUTO_NAME_KEY = "Selected Auto";
  private static final String SELECTED_AUTO_PATHS_KEY = "Selected Auto Paths";

  // The robot's subsystems and commands are defined
  public static DriveTrain driveTrain = new DriveTrain();
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final Field2d autoPreviewField = new Field2d();
  private final Map<Command, String> commandToAutoFile = new IdentityHashMap<>();
  
  // public static Vision visionRight = new Vision("elm_right_cam", new Transform3d(
  //   Units.inchesToMeters(-3.5), Units.inchesToMeters(13), 0.0, 
  //   new Rotation3d(0, Units.degreesToRadians(-20), 0)
  // ));

  public static Vision visionLeft = new Vision(
    "elm_left_cam", new Transform3d(
    Units.inchesToMeters(-3.5), Units.inchesToMeters(-13), 0.0, 
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
    // Add new autos here. Display name is what appears in Elastic/SmartDashboard chooser,
    // auto file name must match src/main/deploy/pathplanner/autos/<name>.auto (without .auto).
    registerAuto("StraightAutotest", "Straight test P", true);
    registerAuto("Rightside90", "Right Side 90", false);

    autoChooser.onChange(this::updateAutoPreviewForCommand);
    SmartDashboard.putData(AUTO_CHOOSER_KEY, autoChooser);
    SmartDashboard.putData(AUTO_PREVIEW_KEY, autoPreviewField);
    updateAutoPreviewForCommand(autoChooser.getSelected());
  }

  private void configureBindings() {
    // controller.a().onTrue(DriveCommands.autoAimMove(()->-controller.getLeftY(),() ->-controller.getRightX()));
    controller.leftBumper().onTrue(driveTrain.zeroGyro());
  }

  public static boolean getA(){
    return controller.a().getAsBoolean();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void registerAuto(String displayName, String autoFileName, boolean isDefault) {
    // Example for a new auto:
    // registerAuto("Center2Piece", "Center 2 Piece", false);
    // Chooser autos come from PathPlanner .auto/.path files, so editing testFollowPath.java
    // alone will not change what this selected autonomous routine does.
    Command autoCommand = new PathPlannerAuto(autoFileName);
    commandToAutoFile.put(autoCommand, autoFileName);
    if (isDefault) {
      autoChooser.setDefaultOption(displayName, autoCommand);
    } else {
      autoChooser.addOption(displayName, autoCommand);
    }
  }

  private void updateAutoPreviewForCommand(Command selectedCommand) {
    if (selectedCommand == null) {
      clearAutoPreview();
      return;
    }

    String autoFileName = commandToAutoFile.get(selectedCommand);
    if (autoFileName == null) {
      clearAutoPreview();
      return;
    }

    try {
      List<String> pathNames = getAutoPathNames(autoFileName);
      List<Pose2d> previewPoses = new ArrayList<>();

      for (String pathName : pathNames) {
        previewPoses.addAll(PathPlannerPath.fromPathFile(pathName).getPathPoses());
      }

      autoPreviewField.getObject(AUTO_PREVIEW_OBJECT_KEY).setPoses(previewPoses);
      SmartDashboard.putString(SELECTED_AUTO_NAME_KEY, autoFileName);
      SmartDashboard.putStringArray(SELECTED_AUTO_PATHS_KEY, pathNames.toArray(new String[0]));
    } catch (IOException | ParseException | RuntimeException e) {
      clearAutoPreview();
      DriverStation.reportError(
          "Failed to load auto preview for " + autoFileName + ": " + e.getMessage(),
          e.getStackTrace());
    }
  }

  private void clearAutoPreview() {
    autoPreviewField.getObject(AUTO_PREVIEW_OBJECT_KEY).setPoses(List.of());
    SmartDashboard.putString(SELECTED_AUTO_NAME_KEY, "");
    SmartDashboard.putStringArray(SELECTED_AUTO_PATHS_KEY, new String[0]);
  }

  private List<String> getAutoPathNames(String autoFileName) throws IOException, ParseException {
    File autoFile = new File(
        Filesystem.getDeployDirectory(),
        "pathplanner/autos/" + autoFileName + ".auto");

    JSONObject autoJson;
    try (BufferedReader reader = new BufferedReader(new FileReader(autoFile))) {
      autoJson = (JSONObject) new JSONParser().parse(reader);
    }

    Set<String> pathNames = new LinkedHashSet<>();
    collectPathNames(autoJson, pathNames);
    return new ArrayList<>(pathNames);
  }

  private void collectPathNames(Object node, Set<String> pathNames) {
    if (node instanceof JSONObject jsonObject) {
      Object type = jsonObject.get("type");
      if ("path".equals(type)) {
        Object data = jsonObject.get("data");
        if (data instanceof JSONObject dataObject) {
          Object pathName = dataObject.get("pathName");
          if (pathName instanceof String name) {
            pathNames.add(name);
          }
        }
      }

      for (Object value : jsonObject.values()) {
        collectPathNames(value, pathNames);
      }
      return;
    }

    if (node instanceof JSONArray jsonArray) {
      for (Object value : jsonArray) {
        collectPathNames(value, pathNames);
      }
    }
  }
}
