package frc.team2412.robot.util.auto;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonomousChooser {
	public interface CommandSupplier {
		Command getCommand();
	}

	private final SendableChooser<CommandSupplier> autonomousModeChooser = new SendableChooser<>();

	// auto naming scheme (Kind of jank but would be weird to change it up now so let's keep it
	// consistent)
	// STARTING POSITION
	//		Flat, Charge, Bump
	// followed by ACTIONS (multiple possible)
	//		LeaveCom, Charge, ScoreHigh, ScoreLow, Shoot, Pickup

	public AutonomousChooser() {
		autonomousModeChooser.setDefaultOption(
				"FlatLeaveCom", () -> AutonomousTrajectories.getAutoPathByName("FlatLeaveCom"));
		autonomousModeChooser.addOption(
				"BumpLeaveCom", () -> AutonomousTrajectories.getAutoPathByName("BumpLeaveCom"));
		autonomousModeChooser.addOption(
				"ChargeLeaveComCharge",
				() -> AutonomousTrajectories.getAutoPathByName("ChargeLeaveComCharge"));

		autonomousModeChooser.addOption(
				"BumpScoreHighLeaveCom",
				() -> AutonomousTrajectories.getAutoPathByName("BumpScoreHighLeaveCom"));
		autonomousModeChooser.addOption(
				"FlatScoreHighLeaveCom",
				() -> AutonomousTrajectories.getAutoPathByName("FlatScoreHighLeaveCom"));
		// autonomousModeChooser.addOption(
		// 		"FlatScoreLeaveCom", () -> AutonomousTrajectories.getAutoPathByName("FlatPieceScore"));
		// autonomousModeChooser.addOption(
		// 		"BumpScoreLeaveCom", () -> AutonomousTrajectories.getAutoPathByName("BumpPieceScore"));
		// autonomousModeChooser.addOption( // maybe
		// 		"FlatScoreLeaveCom2", () ->
		// AutonomousTrajectories.getAutoPathByName("FlatScoreLeaveCom2"));
		// autonomousModeChooser.addOption(
		// 		"ChargePreloadSecPiece",
		// 		() -> AutonomousTrajectories.getAutoPathByName("ChargePreloadSecPiece"));
		// autonomousModeChooser.addOption(
		// 		"BumpScoreTwo", () -> AutonomousTrajectories.getAutoPathByName("BumpScoreTwo"));
		autonomousModeChooser.addOption(
				"ChargeScoreHighLeaveComCharge",
				() -> AutonomousTrajectories.getAutoPathByName("ChargeScoreHighLeaveComCharge"));

		autonomousModeChooser.addOption(
				"ChargeScoreHighCharge",
				() -> AutonomousTrajectories.getAutoPathByName("ChargeScoreHighCharge"));

		autonomousModeChooser.addOption(
				"FlatScoreHighLeaveComScoreHighRed",
				() -> AutonomousTrajectories.getAutoPathByName("FlatScoreHighLeaveComScoreHighRed"));

		autonomousModeChooser.addOption(
				"FlatScoreHighLeaveComScoreHighBlue",
				() -> AutonomousTrajectories.getAutoPathByName("FlatScoreHighLeaveComScoreHighBlue"));

		autonomousModeChooser.addOption(
				"BumpScoreHighLeaveComScoreHighRed",
				() -> AutonomousTrajectories.getAutoPathByName("BumpScoreHighLeaveComScoreHighRed"));

		autonomousModeChooser.addOption(
				"BumpScoreHighLeaveComScoreHighBlue",
				() -> AutonomousTrajectories.getAutoPathByName("BumpScoreHighLeaveComScoreHighBlue"));

		autonomousModeChooser.addOption(
				"FlatScoreMidLeaveComScoreLowScoreLowBackwardsRed",
				() ->
						AutonomousTrajectories.getAutoPathByName(
								"FlatScoreMidLeaveComScoreLowScoreLowBackwardsRed"));

		autonomousModeChooser.addOption(
				"FlatScoreMidLeaveComScoreLowScoreLowBackwardsBlue",
				() ->
						AutonomousTrajectories.getAutoPathByName(
								"FlatScoreMidLeaveComScoreLowScoreLowBackwardsBlue"));

		autonomousModeChooser.addOption(
				"DONOTUSEFlatScoreMidLeaveComScoreLowScoreLowBackwardsInside",
				() ->
						AutonomousTrajectories.getAutoPathByName(
								"FlatScoreMidLeaveComScoreLowScoreLowBackwardsInside"));

		autonomousModeChooser.addOption(
				"ChargeShootChargeShoot",
				() -> AutonomousTrajectories.getAutoPathByName("ChargeShootChargeShoot"));

		autonomousModeChooser.addOption(
				"ChargeCharge", () -> AutonomousTrajectories.getAutoPathByName("ChargeCharge"));

		autonomousModeChooser.addOption(
				"DONOTUSEBumpScoreHighLeaveComScoreHigh",
				() -> AutonomousTrajectories.getAutoPathByName("UTBumpScoreHighLeaveComScoreHigh"));

		autonomousModeChooser.addOption(
				"BonkChargeLeaveComCharge",
				() -> AutonomousTrajectories.getAutoPathByName("BonkChargeLeaveComCharge"));

		autonomousModeChooser.addOption(
				"BonkFlat2Score", () -> AutonomousTrajectories.getAutoPathByName("BonkFlat2Score"));
		autonomousModeChooser.addOption(
				"BonkBump2Score", () -> AutonomousTrajectories.getAutoPathByName("BonkBump2Score"));
		// autonomousModeChooser.addOption(
		// 		"ChargePickupBot", () -> AutonomousTrajectories.getAutoPathByName("ChargePickupBot"));
		// autonomousModeChooser.addOption(
		// 		"BumpScoreTwo", () -> AutonomousTrajectories.getAutoPathByName("BumpScoreTwo"));
		// autonomousModeChooser.addOption(
		// 		"FlatScoreThree", () -> AutonomousTrajectories.getAutoPathByName("FlatScoreThree"));

		// There are more paths that have been created in the deploy/pathplanner path, they should work
		// however they have not been tested and thus are not added here yet.
		ShuffleboardTab autonomousTab = Shuffleboard.getTab("Autonomous");

		autonomousTab.add("Choose Auto Mode", autonomousModeChooser).withPosition(0, 0).withSize(2, 1);
	}

	public Command getAuto() {
		System.out.println("Selected auto:" + autonomousModeChooser.getSelected());
		return autonomousModeChooser.getSelected().getCommand();
	}
}
