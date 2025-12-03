package frc.robot.subsystems.vision;

import static frc.robot.Constants.*;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;

public class VisionConstants {
  public static final double AMBIGUITY_CUTOFF = 0.1;
  public static final double Z_ERROR_CUTOFF = 0.5;

  // index 0 -> arducam-1, etc
  public static final Transform3d[] CAMERA_TRANSFORM =
      switch (getRobotType()) {
        case COMP -> new Transform3d[] {
          // arducam-1 (front left)
          new Transform3d(
              new Translation3d(-0.2628327298377271, 0.1838727418375329, 0.2612087616993975),
              new Rotation3d(0.0013437830141532724, -0.18652974839545497, -2.967261311784782)),
          // // arducam-2 (front center)
          new Transform3d(
              new Translation3d(-0.2910820214728092, -0.1838727418375329, 0.2591884269566544),
              new Rotation3d(0.004400589512719636, -0.17817453409346934, 2.9694158567617692))
          // new Transform3d(), new Transform3d()
          // // arducam-3 (front right)
          // new Transform3d(0., 0, 0, new Rotation3d(0, 0, 0)),
          // // arducam-4 (back right)
          // new Transform3d(0., 0, 0, new Rotation3d(0, 0, 0))
        };
        case SIM -> new Transform3d[] {
          // arducam-1 (front left)
          new Transform3d(0., 0, 0, new Rotation3d(0, 0, 0)),
          // arducam-2 (front center)
          new Transform3d(0., 0, 0, new Rotation3d(0, 0, 0)),
          // // arducam-3 (front right)
          // new Transform3d(0., 0, 0, new Rotation3d(0, 0, 0)),
          // // arducam-4 (back right)
          // new Transform3d(0., 0, 0, new Rotation3d(0, 0, 0))
        };
        default -> new Transform3d[0];
      };

  public static final List<TagCountDeviation> TAG_COUNT_DEVIATIONS =
      switch (getRobotType()) {
        default -> List.of(
            // 1 tag
            new TagCountDeviation(
                new UnitDeviationParams(0.2, 0.1, 0.6),
                new UnitDeviationParams(0.3, 0.1, 0.9),
                new UnitDeviationParams(0.5, 0.7, 1.5)),
            // 2 tag
            new TagCountDeviation(
                new UnitDeviationParams(0.35, 0.1, 0.4), new UnitDeviationParams(0.5, 0.7, 1.5)),
            // 3+ tag
            new TagCountDeviation(
                new UnitDeviationParams(0.25, 0.07, 0.25), new UnitDeviationParams(0.15, 1, 1.5)));
      };

  public static final int[] IGNORE_TAGS = {1, 2, 3, 4, 12, 13, 14, 15, 16};
  // public static final int[] IGNORE_TAGS = {}; // removed

  // Fixed AprilTag field layout initialization
  public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT;

  static {
    // logic for dynamically setting the april tag field layout
    AprilTagFieldLayout defaultFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    List<AprilTag> aprilTags = defaultFieldLayout.getTags();
    // remove ignored tags
    aprilTags.removeIf(
        tag -> {
          for (int ignoreTag : IGNORE_TAGS) {
            if (tag.ID == ignoreTag) {
              return true;
            }
          }
          return false;
        });
    APRIL_TAG_FIELD_LAYOUT =
        new AprilTagFieldLayout(
            aprilTags, defaultFieldLayout.getFieldWidth(), defaultFieldLayout.getFieldWidth());
  }

  public static record TagCountDeviation(
      UnitDeviationParams xParams, UnitDeviationParams yParams, UnitDeviationParams thetaParams) {
    protected Matrix<N3, N1> computeDeviation(double averageDistance) {
      return VecBuilder.fill(
          xParams.computeUnitDeviation(averageDistance),
          yParams.computeUnitDeviation(averageDistance),
          thetaParams.computeUnitDeviation(averageDistance));
    }

    public TagCountDeviation(UnitDeviationParams xyParams, UnitDeviationParams thetaParams) {
      this(xyParams, xyParams, thetaParams);
    }
  }

  public static record UnitDeviationParams(
      double distanceMultiplier, double eulerMultiplier, double minimum) {
    private double computeUnitDeviation(double averageDistance) {
      return Math.max(minimum, eulerMultiplier * Math.exp(averageDistance * distanceMultiplier));
    }
  }
}
