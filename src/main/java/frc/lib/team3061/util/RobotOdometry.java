package frc.lib.team3061.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;

/**
 * Singleton class for SwerveDrivePoseEstimator that allows it to be shared by
 * subsystems (drivetrain and vision)
 * 
 * Originally by Team 3061. Modified to be thread safe by Team 2930.
 * 
 * All interactions with odometry, both reading the current pose and adding
 * updates to the pose must go through this class.
 */
public class RobotOdometry {
  private static final RobotOdometry robotOdometry = new RobotOdometry();
  private static int updateCount = 0;
  private SwerveDrivePoseEstimator estimator;
  private SwerveModulePosition[] defaultPositions = new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
  };

  private RobotOdometry() {
    estimator = new SwerveDrivePoseEstimator(
        DrivetrainConstants.KINEMATICS, new Rotation2d(), defaultPositions, new Pose2d());
    updateCount = 0;
  }

  // public static RobotOdometry getInstance() {
  // return robotOdometry;
  // }

  // public SwerveDrivePoseEstimator getPoseEstimator() {
  // return estimator;
  // }

  public static Pose2d getEstimatedPosition() {
    synchronized (robotOdometry) {
      return robotOdometry.estimator.getEstimatedPosition();
    }
  }

  public static void addVisionMeasurement(Pose2d pose2d, double currentResultTimeStamp, Vector<N3> fill) {

    double startTime =Timer.getFPGATimestamp();
    int uc = 0;
    synchronized (robotOdometry) {
      robotOdometry.estimator.addVisionMeasurement(pose2d, currentResultTimeStamp,fill);
      uc = RobotOdometry.updateCount++;
    }

    Logger.getInstance().recordOutput("RobotOdometry/elapsedTime", Timer.getFPGATimestamp() - startTime);
    Logger.getInstance().recordOutput("RobotOdometry/updateCount", uc);

  }



  // TODO: add the following
  // SwerveDrivePoseEstimator.resetPosition(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d poseMeters)
  
  // edu.wpi.first.math.estimator.SwerveDrivePoseEstimator.updateWithTime(double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions)



      //  // If out of bounds, clamp to field
      //  // TODO: use field constants!
      //  if (poseEstimator.getEstimatedPosition().getY() <= 0.43) {
      //   poseEstimator.resetPosition(
      //       this.getRotation(),
      //       swerveModulePositions,
      //       new Pose2d(
      //           poseEstimator.getEstimatedPosition().getX(),
      //           0.46,
      //           poseEstimator.getEstimatedPosition().getRotation()));
      // }

      // if (poseEstimator.getEstimatedPosition().getY() > 8.35) {
      //   poseEstimator.resetPosition(
      //       this.getRotation(),
      //       swerveModulePositions,
      //       new Pose2d(
      //           poseEstimator.getEstimatedPosition().getX(),
      //           7.73,
      //           poseEstimator.getEstimatedPosition().getRotation()));
      // }

      // if (DriverStation.getAlliance() == Alliance.Blue) {
      //   if (poseEstimator.getEstimatedPosition().getX() > 15.82
      //       || poseEstimator.getEstimatedPosition().getX() < 0.9) {
      //     poseEstimator.resetPosition(
      //         this.getRotation(),
      //         swerveModulePositions,
      //         new Pose2d(
      //             MathUtil.clamp(poseEstimator.getEstimatedPosition().getX(), 1.81, 15.82),
      //             poseEstimator.getEstimatedPosition().getY(),
      //             poseEstimator.getEstimatedPosition().getRotation()));
      //   }
      // } else {
      //   if (poseEstimator.getEstimatedPosition().getX() > 15.6
      //       || poseEstimator.getEstimatedPosition().getX() < 0.71) {
      //     poseEstimator.resetPosition(
      //         this.getRotation(),
      //         swerveModulePositions,
      //         new Pose2d(
      //             MathUtil.clamp(poseEstimator.getEstimatedPosition().getX(), 0.71, 14.71),
      //             poseEstimator.getEstimatedPosition().getY(),
      //             poseEstimator.getEstimatedPosition().getRotation()));
      //   }
      // }

      // // log poses, 3D geometry, and swerve module states, gyro offset
      // poseEstimatorPose = poseEstimator.getEstimatedPosition();
 
}
