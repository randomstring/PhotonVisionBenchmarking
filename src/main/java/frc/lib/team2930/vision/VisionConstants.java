package frc.lib.team2930.vision;
public final class VisionConstants {
  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private VisionConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  // to get field layout call this:
  //    AprilTagFields.k2023ChargedUp.loadAprilTagFieldLayout();

  // TODO: probably want to increase this a little
  public static final double MAXIMUM_AMBIGUITY = 0.08;

  public static final double MAX_VALID_DISTANCE_AWAY_METERS = 3.0;
}
