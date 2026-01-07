package frc.robot.subsystems.swerve.controllers;

import static frc.robot.subsystems.swerve.DriveConstants.PID_AUTOALIGN_CONSTANTS;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;
import frc.robot.RobotState;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class PIDAutoAlignController {

  // supplies the position values
  private ProfiledPIDController magController;
  private Supplier<Pose2d> positionSupplier;

  // target position
  private Pose2d targetPosition;
  private Pose2d startPosition;
  private double xVel;
  private double yVel;
  private final Supplier<Rotation2d> yawSupplier;
  private final Supplier<Translation2d> velocity;
  private double time;

  // private double startVel;

  private TrapezoidProfile testTrapezoidProfile;

  public PIDAutoAlignController(
      Supplier<Pose2d> positionSupplier, Supplier<Rotation2d> yawSupplier, Pose2d targetPosition) {

    this.positionSupplier = positionSupplier;
    this.yawSupplier = yawSupplier;
    this.targetPosition = targetPosition;
    this.velocity = () -> RobotState.getInstance().getVelocity();

    // setting up the ProfiledPIDController
    magController =
        new ProfiledPIDController(
            PID_AUTOALIGN_CONSTANTS.kP(),
            PID_AUTOALIGN_CONSTANTS.kI(),
            PID_AUTOALIGN_CONSTANTS.kD(),
            new Constraints(
                PID_AUTOALIGN_CONSTANTS.maxVelocity(), PID_AUTOALIGN_CONSTANTS.maxAcceleration()),
            Constants.PERIODIC_LOOP_SEC);
    setTargetPosition(targetPosition);
    magController.disableContinuousInput();
    magController.setTolerance(0, 0);

    testTrapezoidProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(5, 10));
  }
  // calculate how to get to the desired position
  public void calculateLinearMovement() {
    double currToTargDy = positionSupplier.get().getY() - targetPosition.getY();
    double currToTargDx = positionSupplier.get().getX() - targetPosition.getX();
    Rotation2d currToTargAngle = new Rotation2d(Math.atan2(currToTargDy, currToTargDx));

    double startToTargDy = startPosition.getY() - targetPosition.getY();
    double startToTargDx = startPosition.getX() - targetPosition.getX();
    Rotation2d startToTargAngle = new Rotation2d(Math.atan2(startToTargDy, startToTargDx));

    double startToCurrDy = startPosition.getY() - positionSupplier.get().getY();
    double startToCurrDx = startPosition.getX() - positionSupplier.get().getX();
    // probably could have calculated this with triangle stuff... welp
    Rotation2d startToCurrAngle = new Rotation2d(Math.atan2(startToCurrDy, startToCurrDx));

    // the naming is very important
    double magTranslCurrPos =
        Math.hypot(
                positionSupplier.get().getX() - startPosition.getX(),
                positionSupplier.get().getY() - startPosition.getY())
            * (Math.abs(startToTargAngle.minus(startToCurrAngle).getRadians()) > Math.PI / 2
                ? -1
                : 1);
    double magTanslTargPos =
        Math.hypot(
            targetPosition.getX() - startPosition.getX(),
            targetPosition.getY() - startPosition.getY());

    double pidOutput = magController.calculate(magTranslCurrPos, magTanslTargPos);
    double magVel = pidOutput + magController.getSetpoint().velocity;
    magVel = (Math.abs(magVel) < 0.02 ? 0 : magVel);
    yVel =
        magVel
            * currToTargAngle.getSin()
            * (Math.abs(currToTargAngle.minus(startToTargAngle).getRadians()) > Math.PI / 2
                ? 1
                : -1);
    xVel =
        magVel
            * currToTargAngle.getCos()
            * (Math.abs(currToTargAngle.minus(startToTargAngle).getRadians()) > Math.PI / 2
                ? 1
                : -1);
    if (positionSupplier.get().getTranslation().getDistance(targetPosition.getTranslation())
        < 0.01) {
      xVel = 0;
      yVel = 0;
    }
    Logger.recordOutput("Swerve/PIDAutoalign/Angle", currToTargAngle);
    Logger.recordOutput("Swerve/PIDAutoalign/OriginAngle", startToTargAngle);
    Logger.recordOutput("Swerve/PIDAutoalign/SetpointPos", magController.getSetpoint().position);
    Logger.recordOutput("Swerve/PIDAutoalign/CurrPos", magTranslCurrPos);
    Logger.recordOutput("Swerve/PIDAutoalign/TargPos", magTanslTargPos);
    Logger.recordOutput("Swerve/PIDAutoalign/magVel", magVel);
    Logger.recordOutput("Swerve/PIDAutoalign/Target", targetPosition);
    Logger.recordOutput("Swerve/PIDAutoalign/TrapVel", magController.getSetpoint().velocity);
    Logger.recordOutput("Swerve/PIDAutoalign/PIDVel", pidOutput);
  }

  public void calculateLinearMovementTest() {
    double magTranslCurrPos = positionSupplier.get().getX() - startPosition.getX();
    double magTanslTargPos = targetPosition.getX() - startPosition.getX();
    double magVel = magController.calculate(magTranslCurrPos);
    xVel = magVel + magController.getSetpoint().velocity;
    // xVel = magVel - startVel;
    yVel = 0;
    // Logger.recordOutput("Swerve/PIDAutoalign/TestMagVel", -magVel);
    // Logger.recordOutput("Swerve/PIDAutoalign/TestTrapVel",
    // -magController.getSetpoint().velocity);
    // Logger.recordOutput("Swerve/PIDAutoalign/TestTarget", targetPosition);
    // Logger.recordOutput("Swerve/PIDAutoalign/TestTargPos", magTanslTargPos);
    // Logger.recordOutput(
    //     "Swerve/PIDAutoalign/TestSetpointPos", magController.getSetpoint().position);
    // Logger.recordOutput("Swerve/PIDAutoalign/TestCurrPos", magTranslCurrPos);
    time += Constants.PERIODIC_LOOP_SEC;
  }

  public double calculateTimeLeft() {
    double totalTime;
    double d = startPosition.getTranslation().getDistance(targetPosition.getTranslation());
    double a = PID_AUTOALIGN_CONSTANTS.maxAcceleration();
    double v = PID_AUTOALIGN_CONSTANTS.maxVelocity();
    if (d - a * (Math.pow((v / a), 2)) > 0) {
      totalTime = (d - (a * (v / a) * (v / a))) / v + 2 * (v / a);
    } else {
      totalTime = 2 * Math.sqrt(d / a);
    }
    double timeLeft =
        totalTime
            * (positionSupplier.get().getTranslation().getDistance(targetPosition.getTranslation())
                / d);
    Logger.recordOutput("Swerve/PIDAutoalign/TimeLeft", totalTime);
    return timeLeft;
  }

  // update the values
  public ChassisSpeeds update() {
    calculateLinearMovement();
    Logger.recordOutput("Swerve/PIDAutoalign/XVel", xVel);
    Logger.recordOutput("Swerve/PIDAutoalign/YVel", yVel);
    return ChassisSpeeds.fromFieldRelativeSpeeds(-xVel, -yVel, 0, yawSupplier.get());
  }
  // log your data in advantage kit
  public Pose2d getTargetPosition() {
    return targetPosition;
  }

  public double getXVel() {
    return -xVel;
  }

  public double getYVel() {
    return -yVel;
  }

  public void setTargetPosition(Pose2d targetPosition) {
    startPosition = positionSupplier.get();
    this.targetPosition = targetPosition;
    double magTranslCurrPos =
        Math.hypot(
            positionSupplier.get().getX() - startPosition.getX(),
            positionSupplier.get().getY() - startPosition.getY());
    double magTanslTargPos =
        Math.hypot(
            targetPosition.getX() - startPosition.getX(),
            targetPosition.getY() - startPosition.getY());
    magController.setGoal(magTanslTargPos);
    magController.reset(magTranslCurrPos, calculateForwardVelocity());
    time = 0;
    // startVel = calculateForwardVelocity();
  }

  public double calculateForwardVelocity() {
    Translation2d vel = velocity.get();
    double x = vel.getX();
    double y = vel.getY();
    Pose2d relativeTargetPosition = targetPosition.relativeTo(positionSupplier.get());
    Rotation2d targetAngle =
        new Rotation2d(Math.atan2(relativeTargetPosition.getY(), relativeTargetPosition.getX()));
    Rotation2d currentVelAngle = new Rotation2d(Math.atan2(y, x));
    Rotation2d angleDiff = targetAngle.minus(currentVelAngle);
    double forwardVelocity = Math.cos(angleDiff.getRadians()) * vel.getNorm();
    System.out.println("Forward Velocity: " + vel.getX());
    System.out.println("Angle Diff: " + angleDiff.getDegrees());
    return forwardVelocity;
  }
}
