// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Rotation;

import java.io.File;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.dyn4j.geometry.Rotation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

@Logged
public class SwerveSubsystem extends SubsystemBase {

  SwerveDrive swerveDrive;

  double maximumSpeed = Units.feetToMeters(14.5);

  public SwerveSubsystem(File directory) {

    boolean blueAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Blue;
    Pose2d startingPose = blueAlliance ? new Pose2d(new Translation2d(Meter.of(1),
                                                                      Meter.of(4)),
                                                    Rotation2d.fromDegrees(0))
                                       : new Pose2d(new Translation2d(Meter.of(16),
                                                                      Meter.of(4)),
                                                    Rotation2d.fromDegrees(180));

    // Swerve Data and Telemetry
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
      // Alternative method if you don't want to supply the conversion factor via JSON files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(true,
                                               true,
                                               0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(false,
                                                1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
    // swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible
  }

  public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
    swerveDrive = new SwerveDrive(
      driveCfg, 
      controllerCfg, 
      maximumSpeed, 
      new Pose2d(new Translation2d(Meter.of(2), Meter.of(0)),
      Rotation2d.fromDegrees(0)));
  }

  // for sysid tuning :)
  public Command sysIdDriveMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
      SwerveDriveTest.setAngleSysIdRoutine(
      new Config(), 
      this, swerveDrive), 
      3.0, 5.0, 3.0);
  }

  // also for sysid tuning :)
  public Command sysIdAngleMotorCommand() {
    return SwerveDriveTest.generateSysIdCommand(
      SwerveDriveTest.setAngleSysIdRoutine(
        new Config(),
        this, swerveDrive), 
        3.0, 5.0, 3.0);
  }

  // reset/centre all the swerve modules
  public Command centreModulesCommand() {
    return run(() -> Arrays.asList(swerveDrive.getModules())
    .forEach(hello -> hello.setAngle(0.0))); // hello being each individual module ruhahah
  }

  // simple drive forward command
  public Command driveForward() {
    return run (()-> {
      swerveDrive.drive(new Translation2d(1, 0), 0, false, false);
    }).finallyDo(() -> {
      swerveDrive.drive(new Translation2d(0, 0), 0, false, false); // finally do when command ends this will run once (stops the translation basically)
    });
  }

  // replaces swerve module ff with a new simple motor ff object
  public void replaceSwerveModuleFeedforward(double ks, double kv, double ka) {
    swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(ks, kv, ka));
  }

  // not so simple drive using given speeds / controller inputs
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    return run(() -> {
      swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
        translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
        translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()
      ), 0.8), Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
      true,
      false);
    });
  }

  // use rotation headings as a setpoint instead of controller inputs
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY) {
    return run (() -> {

      Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(
        translationX.getAsDouble(),
        translationY.getAsDouble()
      ), 0.8); // this 0.8 value is your max value, it interpolates the given translation values (0 to 1) to that given value, so scalar = 0.8, then 1 = 0.8, 0.5 = 0.4 etc.

      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(), 
        scaledInputs.getY(), 
        headingX.getAsDouble(), 
        headingY.getAsDouble(), 
        swerveDrive.getMaximumChassisVelocity()));

    });
  }
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.driveFieldOriented(velocity.get());
    });
  }

  // primary control method:
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(
      translation,
      rotation,
      fieldRelative,
      false);
  }
  // drive according to the cahssis robot oriented velocity
  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  // provides swerve kinematics
  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  //reset the robot odometry
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  // provide the robot pose/ get robot pose
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  // set chassis speed using closed-loop velocity ctrl
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  // post the trajectory to the field
  public void postTrajectory(Trajectory trajectory) {
    swerveDrive.postTrajectory(trajectory);
  }

  // reset gyro yaw, so odometry position remains the same, but facing towards 0.
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  // get alliance, true if red, false is blue, defaults to blue (false) is none is avaliable
  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  public void zeroGyroWithAlliance() {
    if (isRedAlliance()) {
      zeroGyro();

      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
    }
  }

  // if brake == true, motors are on brake, else motors are on coast
  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  // provides the yaw angle of the robot
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  // provides chassis speeds of the robot based on a controller with 2 joysticks, one for direction, other for yaw angle
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(
      xInput,
      yInput
    ));
    return swerveDrive.swerveController.getTargetSpeeds(
      scaledInputs.getX(), 
      scaledInputs.getY(), 
      headingX, 
      headingY, 
      getHeading().getRadians(),
      maximumSpeed);
  }

  // same as last but with 1 joystick and one twist
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
  {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
    return swerveDrive.swerveController.getTargetSpeeds(
      scaledInputs.getX(), 
      scaledInputs.getY(), 
      angle.getRadians(), 
      getHeading().getRadians(),
      maximumSpeed);
  }

  // provides the current field-relative veolicty (x,y and omega) of the robot
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  // provides current robot velocity
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  // provides the controller given to the subsystem
  public SwerveController getSwerveController() {
    return swerveDrive.getSwerveController();
  }

  // provides the configuration of the swerve drive
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  // lock wheels, points them towards the middle, stops it from moving
  public void lock() {
    swerveDrive.lockPose();
  }

  // gets the navx3's provided pitch
  public Rotation2d getPitch() {
    return swerveDrive.getPitch();
  }

  // returns the ENTIRE swervedrive object
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
