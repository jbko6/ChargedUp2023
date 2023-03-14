package frc.team2412.robot.util.auto;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team2412.robot.subsystems.DrivebaseSubsystem;

public class AutoBuilder {
    private static final PathConstraints CONSTRAINTS = new PathConstraints(1, 1);
    private static final PIDController AUTO_TRANSLATION_PID = new PIDController(0, 0, 0);
    private static final PIDController AUTO_ROTATION_PID = new PIDController(0, 0, 0);

    public enum StartingPosition {
        LEFT(new Pose2d(0,0, Rotation2d.fromDegrees(0))),
        MIDDLE(new Pose2d(0,0, Rotation2d.fromDegrees(0))),
        RIGHT(new Pose2d(0,0, Rotation2d.fromDegrees(0)));

        public Pose2d pose;

        StartingPosition(Pose2d pose) {
            this.pose = pose;
        }
    }

    public enum GameObject {
        CONE,
        CUBE,
        NONE
    }

    public enum ScoringArea {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public enum ScoringPosition {
        HYBRID,
        MIDDLE,
        TOP
    }

    public enum Action {
        LEAVE_COMMUNITY,
        PICKUP,
        SCORE,
        CLIMB
    }

    public static Command buildAuto(DrivebaseSubsystem drivebaseSubsystem, StartingPosition startingPosition, GameObject startingObject, ScoringPosition scoringTypePriority, ScoringArea scoringArea, Action... actions) {
        SequentialCommandGroup commands = new SequentialCommandGroup();

        // init game objects available on field
        HashMap<Pose2d, GameObject> availableGameObjects = new HashMap<Pose2d, GameObject>();
        // repeat for each game object
        availableGameObjects.put(new Pose2d(), GameObject.CONE);
        // when a game object is taken, it is removed
        availableGameObjects.remove(new Pose2d());

        GameObject storedGameObject = startingObject;

        Pose2d currentPosition = startingPosition.pose;
        for (Action action : actions) {
            // Handle leave community case
            if (action == Action.LEAVE_COMMUNITY) {
                // To get a location we need to get to be out of community, we can get our starting pose and add the distance of starting position from the community line
                // Another way to achieve this would be to take the current position and navigate to that position but with a set x value
                Pose2d leaveCommunityPose = startingPosition.pose.plus(new Transform2d(new Translation2d(10, 0), Rotation2d.fromDegrees(0)));
                commands.addCommands(driveToPoint(drivebaseSubsystem, currentPosition, leaveCommunityPose));
            }

            // Handle picking up a game piece
            if (action == Action.PICKUP) {
                // NOTE: with this functionality, the robot will pickup the nearest game object. so the drivers cannot specifiy which game objects to pick, could cause collisions with other robot's paths

                // get the nearest game piece
                Pose2d gamePiecePose = currentPosition.nearest(new ArrayList<Pose2d>(availableGameObjects.keySet()));
                commands.addCommands(driveToPoint(drivebaseSubsystem, currentPosition, gamePiecePose));
                // add commands to pick up the piece idk how to do this

                // remove the game piece from the pool
                availableGameObjects.remove(gamePiecePose);
                store
            }
        }

        return commands;
    }

    private static Command driveToPoint(DrivebaseSubsystem drivebaseSubsystem, Pose2d currentPosition, Pose2d targetPosition) {
        PathPlannerTrajectory traj = PathPlanner.generatePath(CONSTRAINTS, 
            new PathPoint(currentPosition.getTranslation(), Rotation2d.fromRadians(Math.atan2(targetPosition.getY()-currentPosition.getY(), targetPosition.getX()-currentPosition.getX())), currentPosition.getRotation()),
            new PathPoint(targetPosition.getTranslation(), null, targetPosition.getRotation()));
        
        return new PPSwerveControllerCommand(traj, drivebaseSubsystem::getPose, DrivebaseSubsystem.kinematics, AUTO_TRANSLATION_PID, AUTO_TRANSLATION_PID, AUTO_ROTATION_PID, drivebaseSubsystem::drive, true, drivebaseSubsystem);
    }
}