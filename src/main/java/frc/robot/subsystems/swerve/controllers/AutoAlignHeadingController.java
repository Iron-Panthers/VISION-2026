package frc.robot.subsystems.swerve.controllers;

import static frc.robot.subsystems.swerve.DriveConstants.HEADING_CONTROLLER_CONSTANTS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import java.util.function.Supplier;

public class AutoAlignHeadingController extends HeadingController {

  public AutoAlignHeadingController(
      Supplier<Rotation2d> headingSupplier, Rotation2d targetHeading, double timeLeft) {
    super(headingSupplier, targetHeading);
    setTargetHeading(targetHeading, timeLeft);
  }

  @Override
  public void setTargetHeading(Rotation2d targetHeading) {
    setTargetHeading(targetHeading, 0);
  }

  public void setTargetHeading(Rotation2d targetHeading, double timeLeft) {
    super.setTargetHeading(targetHeading);
    double a = HEADING_CONTROLLER_CONSTANTS.maxAcceleration();
    double v = HEADING_CONTROLLER_CONSTANTS.maxVelocity();
    if (a != 0 && v != 0) {
      double adjustedTimeLeft = timeLeft - a * (Math.pow((v / a), 2)) / v;
      Rotation2d adjustedAngleDifference =
          super.getHeadingSupplier()
              .get()
              .minus(targetHeading)
              .minus(
                  new Rotation2d(
                      Math.toRadians(
                          a
                              * (Math.pow(
                                  (v / a), 2))))); // Amount of time to accelerate and decelerate
      v = Math.min(Math.abs(adjustedAngleDifference.getRadians()) / adjustedTimeLeft, 5);
    } else {
      System.out.println("AutoAlignHeadingController: max velocity or acceleration is set to 0");
    }
    super.getController().setConstraints(new Constraints(v, a));
  }
}
