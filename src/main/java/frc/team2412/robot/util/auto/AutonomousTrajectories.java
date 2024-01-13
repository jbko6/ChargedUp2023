package frc.team2412.robot.util.auto;

import static frc.team2412.robot.commands.arm.SetWristCommand.WristPosition.WRIST_PRESCORE;
import static frc.team2412.robot.subsystems.ArmSubsystem.ArmConstants.PositionType.*;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team2412.robot.Robot;
import frc.team2412.robot.Subsystems;
import frc.team2412.robot.commands.arm.SetFullArmCommand;
import frc.team2412.robot.commands.arm.SetWristAngleCommand;
import frc.team2412.robot.commands.arm.SetWristCommand;
import frc.team2412.robot.commands.arm.SetWristCommand.WristPosition;
import frc.team2412.robot.commands.autonomous.AutoBalanceCommand;
import frc.team2412.robot.commands.intake.IntakeSetFastOutCommand;
import frc.team2412.robot.commands.intake.IntakeSetInCommand;
import frc.team2412.robot.commands.intake.IntakeSetOutCommand;
import frc.team2412.robot.subsystems.DrivebaseSubsystem;

import java.util.HashMap;
import java.util.List;

public class AutonomousTrajectories {
	private static final Subsystems s = Robot.getInstance().subsystems;

	public static Command getAutoPathByName(String name) {



		List<PathPlannerTrajectory> pathGroup =
				PathPlanner.loadPathGroup(name, new PathConstraints(2.0, 2.0));
		HashMap<String, Command> eventMap = new HashMap<String, Command>();
		eventMap.put(
				"AutoBalance", new AutoBalanceCommand(Robot.getInstance().subsystems.drivebaseSubsystem));
		if (!(s.intakeSubsystem == null) && !(s.armSubsystem == null)) {
			Command wristOut = new SetWristCommand(s.armSubsystem, WristPosition.WRIST_SCORE);
			Command wristPrescore = new SetWristCommand(s.armSubsystem, WristPosition.WRIST_PRESCORE);
			Command wristShoot = new SetWristAngleCommand(s.armSubsystem, 0.34);
			Command intakeOut = new IntakeSetOutCommand(s.intakeSubsystem).withTimeout(0.5);
			Command intakeFastOut = new IntakeSetFastOutCommand(s.intakeSubsystem).withTimeout(0.1);
			Command intakeIn = new IntakeSetInCommand(s.intakeSubsystem).withTimeout(0.2);
			Command wristIn = new SetWristCommand(s.armSubsystem, WristPosition.WRIST_RETRACT);
			Command scoreBottom =
					new SequentialCommandGroup(intakeIn, wristPrescore, intakeOut.withTimeout(1.5), wristIn);
			Command armLow = new SetFullArmCommand(s.armSubsystem, ARM_LOW_POSITION, WRIST_PRESCORE);
			Command armMid = new SetFullArmCommand(s.armSubsystem, ARM_MIDDLE_POSITION, WRIST_PRESCORE);
			Command armHigh = new SetFullArmCommand(s.armSubsystem, ARM_HIGH_POSITION, WRIST_PRESCORE);
			Command armSubstation =
					new SetFullArmCommand(s.armSubsystem, ARM_SUBSTATION_POSITION, WRIST_PRESCORE);
			Command stow =
					new SetFullArmCommand(s.armSubsystem, ARM_LOW_POSITION, WristPosition.WRIST_RETRACT, 0.5);
			Command scoreHigh =
					new SequentialCommandGroup(
							new SetFullArmCommand(s.armSubsystem, ARM_HIGH_POSITION, WristPosition.WRIST_SCORE),
							new WaitCommand(0.2),
							new SetFullArmCommand(
									s.armSubsystem, ARM_SUBSTATION_POSITION, WristPosition.WRIST_PRESCORE),
							new WaitCommand(0.3),
							new IntakeSetOutCommand(s.intakeSubsystem).withTimeout(0.1));

			eventMap.put("ScoreBottom", scoreBottom);
			eventMap.put("ScoreHigh", scoreHigh);

			eventMap.put("WristRetract", wristIn);
			eventMap.put("WristPrescore", wristPrescore);
			eventMap.put("WristScore", wristOut);
			eventMap.put("WristShoot", wristShoot);
			eventMap.put("IntakeOut", intakeOut);
			eventMap.put("IntakeFastOut", intakeFastOut);
			eventMap.put("IntakeIn", intakeIn);
			eventMap.put("ArmHigh", armHigh);
			eventMap.put("ArmLow", armLow);
			eventMap.put("ArmMid", armMid);
			eventMap.put("ArmSubstation", armSubstation);
			eventMap.put("Stow", stow);
			eventMap.put("Wait", new WaitCommand(0.5));
		}
		Command fullAuto = Robot.getInstance().getAutoBuilder(eventMap).fullAuto(pathGroup);
		return fullAuto;
	}
	public Command getChoreoAutoPathByName(DrivebaseSubsystem drivebaseSubsystem,String pathName){
		PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathName);
		return new FollowPathHolonomic(path, drivebaseSubsystem::getPose, drivebaseSubsystem::getChassisSpeeds, drivebaseSubsystem::drive, new HoloniomicPathFollowerConfig(new PIDConstants(5.0, 0.0., 0.0), new PIDConstants(5.0, 0.0, 0.0.), 4.5, 0.4 new ReplanningConfig()), () -> {var aliiance = DriverStation.getAlliance(); if (alliance.isPresent()){return alliance.get() == DriverStation.Alliance.Red;} return false;}, this);
	
	// new FollowPathHolonomic(alignmentPath, drivebaseSubsystem::getPose, drivebaseSubsystem::getChassisSpeeds, null, pathFollowerConfig, DriverAssist::isRedAlliance, drivebaseSubsystem);
	}
}
