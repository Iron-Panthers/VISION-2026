package frc.robot.subsystems.swerve.controllers.heading;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.DriveConstants.HeadingControllerConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TeleopHeadingController extends BaseHeadingController {

  public TeleopHeadingController(
      Supplier<Rotation2d> headingSupplier,
      Rotation2d targetHeading,
      HeadingControllerConstants headingControllerConstants) {
    super(headingSupplier, targetHeading, headingControllerConstants);
  }

  public double update() {
    double output = super.update();
    Logger.recordOutput(
        "Swerve/HeadingController/SetpointVelocity", controller.getSetpoint().velocity);
    Logger.recordOutput("Swerve/HeadingController/Output", output);
    Logger.recordOutput(
        "Swerve/HeadingController/SetpointPosition", controller.getSetpoint().position);
    Logger.recordOutput(
        "Swerve/HeadingController/CurrentPosition", headingSupplier.get().getRadians());
    return output;
  }
}
