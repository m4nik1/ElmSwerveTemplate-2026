package frc.robot.autoCommands;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class CenterAuto {

    private final String pathName = "Center Auto";

    public PathPlannerAuto getAuto()  throws IOException, ParseException {
        var path1 = PathPlannerPath.fromPathFile(pathName);

        // Get starting pose of the path
        var startingPose = new Pose2d(path1.getPoint(0).position, path1.getIdealStartingState().rotation());
        

        // We define the command we want to run in auto
        Command cmd = Commands.sequence(
            AutoBuilder.resetOdom(startingPose),
            // Instant command can be run here for setting intake down
            AutoBuilder.followPath(path1)   ,
            new WaitCommand(1.0) // Shoot
        );

        return new PathPlannerAuto(cmd, startingPose);
    } 
}
