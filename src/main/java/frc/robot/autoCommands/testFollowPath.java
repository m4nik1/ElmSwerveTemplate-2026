// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autoCommands;

import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import org.json.simple.parser.ParseException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Add your docs here. */
public class testFollowPath {

    private final String path1Name= "Right Side Auto";


    // Use deadline commands to run a intake or shooter while following a path
    public PathPlannerAuto getAuto()  throws IOException, ParseException {
        var path1 = PathPlannerPath.fromPathFile(path1Name);

        // Get starting pose of the path
        var startingPose = new Pose2d(path1.getPoint(0).position, path1.getIdealStartingState().rotation());
        

        // We define the command we want to run in auto
        Command cmd = Commands.sequence(
            AutoBuilder.resetOdom(startingPose),
            AutoBuilder.followPath(path1)
        );

        return new PathPlannerAuto(cmd, startingPose);
    }
}
