package frc.robot.subsystems.swerve.controllers.heading;

import static frc.robot.subsystems.swerve.DriveConstants.HEADING_CONTROLLER_CONSTANTS;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.DriveConstants.HeadingControllerConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public abstract class BaseHeadingController {
  // the PID controller
  protected ProfiledPIDController controller;
  protected Supplier<Rotation2d> headingSupplier;
  protected Rotation2d targetHeading;

  public BaseHeadingController(
      Supplier<Rotation2d> headingSupplier,
      Rotation2d targetHeading,
      HeadingControllerConstants headingControllerConstants) {
    this.headingSupplier = headingSupplier;
    this.targetHeading = targetHeading;

    // setting the following controller
    controller =
        new ProfiledPIDController(
            headingControllerConstants.kP(),
            0,
            headingControllerConstants.kD(),
            new Constraints(
                headingControllerConstants.maxVelocity(),
                headingControllerConstants.maxAcceleration()),
            Constants.PERIODIC_LOOP_SEC);

    controller.setTolerance(Units.degreesToRadians(headingControllerConstants.tolerance()));
    controller.enableContinuousInput(-Math.PI, Math.PI);
    controller.reset(headingSupplier.get().getRadians());
  }

  /**
   * Called every 20 milliseconds to calculate the output Omega Radians Per Second
   *
   * @return omega radians per second of the heading controller
   */
  public double update() {
    double output =
        controller.calculate(headingSupplier.get().getRadians() - targetHeading.getRadians(), 0)
            + controller.getSetpoint().velocity;
    return Math.abs(output) > 0.02 ? output : 0; // To prevent jittering
  }

  /** Weather or not the controller is close enough to its target */
  @AutoLogOutput(key = "Swerve/HeadingController/AtTarget")
  public boolean atTarget() {
    return epsilonEquals(
        controller.getSetpoint().position,
        controller.getGoal().position,
        Units.degreesToRadians(HEADING_CONTROLLER_CONSTANTS.tolerance()));
  }

  protected boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  /**
   * Setting the target heading
   *
   * @param targetHeading
   */
  public void setTargetHeading(Rotation2d targetHeading) {
    this.targetHeading = targetHeading;
  }

  // -- Getter methods --

  /**
   * @return Target heading of the controller
   */
  public Rotation2d getTargetHeading() {
    return targetHeading;
  }

  /**
   * @return Profiled PID controller
   */
  protected ProfiledPIDController getController() {
    return controller;
  }

  protected Supplier<Rotation2d> getHeadingSupplier() {
    return headingSupplier;
  }
}
