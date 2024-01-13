package frc.team2412.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.sim.PhysicsSim;
import java.io.File;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class DrivebaseSubsystem extends SubsystemBase {

	public static final SwerveDriveKinematics kinematics =
			new SwerveDriveKinematics(
					new Translation2d(8.5, 8.5),
					new Translation2d(-8.5, 8.5),
					new Translation2d(8.5, -8.5),
					new Translation2d(-8.5, -8.5));
	public static final double MAX_SPEED = 4.1148;
	public static final Rotation2d MAX_ROTATIONS_PER_SEC = Rotation2d.fromRotations(1.0724);

	private final SwerveDrive swerveDrive;

	public DrivebaseSubsystem(SwerveDrivePoseEstimator poseEstimator, Field2d field) {

		File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "bonkswerve");

		try {
			swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(MAX_SPEED);
		} catch (Exception e) {
			throw new RuntimeException();
		}

		swerveDrive.setMotorIdleMode(true);
		swerveDrive.setModuleStateOptimization(true);
		swerveDrive.setHeadingCorrection(true);
	}

	public Pose2d getPose() {
		return swerveDrive.getPose();
	}

	public void resetPose(Pose2d pose) {
		swerveDrive.resetOdometry(pose);
	}

	public void resetPose() {
		resetPose(new Pose2d());
	}

	public SwerveDriveKinematics getKinematics() {
		return swerveDrive.kinematics;
	}

	public void drive(
			double forward,
			double strafe,
			Rotation2d rotation,
			boolean fieldOriented,
			boolean autoBalance) {
		// TODO: add auto balance
		swerveDrive.drive(
				new Translation2d(forward, strafe), rotation.getRadians(), fieldOriented, false);
	}

	public void drive(ChassisSpeeds speeds) {
		swerveDrive.drive(speeds);
	}

	public void toggleXWheels() {
		// TODO: implement this
	}

	public double getVelocity() {
		return Math.sqrt(
				Math.pow(swerveDrive.getFieldVelocity().vxMetersPerSecond, 2)
						+ Math.pow(swerveDrive.getFieldVelocity().vyMetersPerSecond, 2));
	}

	public void stopAllMotors() {
		// TODO: implement this
	}

	public void resetGyroAngle(Rotation2d orientation) {
		swerveDrive.setGyro(new Rotation3d(0, 0, orientation.unaryMinus().getRadians()));
	}

	public void resetGyroAngle() {
		swerveDrive.zeroGyro();
	}

	public void resetGyroAngleWithOrientation(Rotation2d orientation) {
		swerveDrive.setGyro(new Rotation3d(0, 0, swerveDrive.getYaw().plus(orientation).getRadians()));
	}

	public void disableNoMotionCalibration() {
		// TODO: should this do something?
	}

	public void enableNoMotionCalibration() {
		// TODO: should this do something?
	}

	public void setUseVisionMeasurements(boolean useVisionMeasurements) {
		// TODO: add back logging stuff
		// useVisionMeasurementsPublisher.set(useVisionMeasurements);
	}

	public void simInit(PhysicsSim sim) {
		// TODO: does this need to do anything?
	}

	public SwerveModuleState[] getCurrentStates() {
		return swerveDrive.getStates();
	}

	public ChassisSpeeds getChassisSpeeds() {
		return kinematics.toChassisSpeeds(getCurrentStates());
	}

	public Rotation2d getChassisRotation() {
		return swerveDrive.getYaw();

		// might need +orienttation?
	}
	public double getDrivebaseRadius() {
		return swerveDrive.rad
	}
}
