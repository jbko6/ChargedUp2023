package frc.team2412.robot.util;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.GoalEndState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team2412.robot.subsystems.DrivebaseSubsystem;
import frc.team2412.robot.subsystems.IntakeSubsystem.IntakeConstants.GamePieceType;
import static frc.team2412.robot.subsystems.DrivebaseSubsystem.MAX_ROTATIONS_PER_SEC;
import java.util.List;

public class DriverAssist {
	private static final PIDController ASSIST_TRANSLATION_PID = new PIDController(10.0, 0, 0);
	private static final PIDController ASSIST_ROTATION_PID = new PIDController(8.0, 0, 0);
	private static final PathConstraints ASSIST_CONSTRAINTS = new PathConstraints(4.0, 2.0, MAX_ROTATIONS_PER_SEC.getRadians(), MAX_ROTATIONS_PER_SEC.getRadians()/2);



	// 4.0 max vel
	// 2.0 max accel

	// comments are relative to the driver, with numbers increasing from right to left
	private static final Pose2d[] CUBE_ALIGNMENT_POSES = {
		new Pose2d(new Translation2d(1.90, 1.05), Rotation2d.fromDegrees(180)), // 1st cube scoring area
		new Pose2d(new Translation2d(1.90, 2.75), Rotation2d.fromDegrees(180)), // 2nd cube scoring area
		new Pose2d(new Translation2d(1.90, 4.43), Rotation2d.fromDegrees(180)), // 3rd cube scoring area
	};

	private static final Pose2d[] CONE_ALIGNMENT_POSES = {
		new Pose2d(new Translation2d(1.80, 0.5), Rotation2d.fromDegrees(180)), // 1st cone scoring area
		new Pose2d(new Translation2d(1.80, 1.6), Rotation2d.fromDegrees(180)), // 2nd cone scoring area
		new Pose2d(new Translation2d(1.80, 2.18), Rotation2d.fromDegrees(180)), // 3rd cone scoring area
		new Pose2d(new Translation2d(1.80, 3.3), Rotation2d.fromDegrees(180)), // 4th cone scoring area
		new Pose2d(new Translation2d(1.80, 3.86), Rotation2d.fromDegrees(180)), // 5th cone scoring area
		new Pose2d(new Translation2d(1.80, 5), Rotation2d.fromDegrees(180)), // 6th cone scoring area
	};

	private static boolean isRedAlliance() {
		return DriverStation.getAlliance() == Alliance.Red;
	}

	private static Command alignRobot(
			DrivebaseSubsystem drivebaseSubsystem, GamePieceType gamePieceType) {
		Pose2d currentPose = drivebaseSubsystem.getPose();
		Pose2d alignmentPose =
				currentPose.nearest(
						List.of(
								(gamePieceType == GamePieceType.CUBE)
										? CUBE_ALIGNMENT_POSES
										: CONE_ALIGNMENT_POSES));

		// PathPlannerTrajectory alignmentTraj =
		// 		PathPlanner.generatePath(
		// 				ASSIST_CONSTRAINTS,
		// 				new PathPoint(
		// 						currentPose.getTranslation(),
		// 						Rotation2d.fromRadians(
		// 								Math.atan2(	
		// 										alignmentPose.getY() - currentPose.getY(),
		// 										alignmentPose.getX() - currentPose.getX())),
		// 						currentPose.getRotation(),
		// 						drivebaseSubsystem.getVelocity()),
		// 				new PathPoint(
		// 						alignmentPose.getTranslation(),
		// 						Rotation2d.fromDegrees(180),
		// 						alignmentPose.getRotation()));

		ChassisSpeeds currentChassisSpeeds = drivebaseSubsystem.getChassisSpeeds();
		PathPlannerPath  alignmentPath = new PathPlannerPath(List.of(currentPose.getTranslation(), alignmentPose.getTranslation()), ASSIST_CONSTRAINTS, new GoalEndState(0, alignmentPose.getRotation()));
		// goal end state might be bad :(
		
		PathPlannerTrajectory alignmentTraj = new PathPlannerTrajectory(alignmentPath, currentChassisSpeeds, drivebaseSubsystem.getChassisRotation() );


		HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(drivebaseSubsystem.MAX_SPEED, drivebaseSubsystem.get);

		Command newFollowAlignmentCommand = new FollowPathHolonomic(alignmentPath, drivebaseSubsystem::getPose, drivebaseSubsystem::getChassisSpeeds, null, pathFollowerConfig, DriverAssist::isRedAlliance, drivebaseSubsystem);



		Command followAlignmentCommand =
				new PPSwerveControllerCommand(
						alignmentTraj,
						drivebaseSubsystem::getPose,
						DrivebaseSubsystem.kinematics,
						ASSIST_TRANSLATION_PID,
						ASSIST_TRANSLATION_PID,
						ASSIST_ROTATION_PID,
						drivebaseSubsystem::drive,
						true,
						drivebaseSubsystem);

		return followAlignmentCommand;
	}

	public static Command alignRobotCommand(
			DrivebaseSubsystem drivebaseSubsystem, GamePieceType gamePieceType) {
		return new AutoAlignCommand(drivebaseSubsystem, gamePieceType);
	}

	private static class AutoAlignCommand extends CommandBase {
		private final DrivebaseSubsystem drivebaseSubsystem;
		private final GamePieceType gamePieceType;
		private Command alignCommand = null;

		public AutoAlignCommand(DrivebaseSubsystem drivebaseSubsystem, GamePieceType gamePieceType) {
			this.drivebaseSubsystem = drivebaseSubsystem;
			this.gamePieceType = gamePieceType;
		}

		@Override
		public void initialize() {
			System.out.println("Starting new align robot command");
			alignCommand = alignRobot(drivebaseSubsystem, gamePieceType);
			alignCommand.initialize();
		}

		@Override
		public void execute() {
			alignCommand.execute();
		}

		@Override
		public boolean isFinished() {
			return alignCommand.isFinished();
		}

		@Override
		public void end(boolean interrupted) {
			if (interrupted && alignCommand != null) {
				alignCommand.end(true);
			}
			alignCommand = null;
		}
	}
}
