// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionConstants;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Vision visionSystem = new Vision("cammy");
    private final ProfiledPIDController rotationController = new ProfiledPIDController(2, 0, 6, new Constraints(2, 1));


    int targetId = -1;

    // Choreo AutoFactory

    private final AutoFactory autoFactory;

    private double rotationalAdjustment = 0;
    private double xForwardAdjustment = 0;
    private double yForwardAdjustment = 0;
  
    public RobotContainer() {
        autoFactory = new AutoFactory(
            drivetrain::getPose,
            drivetrain::resetPose, 
            drivetrain::followPath, 
            true, 
            drivetrain
        );
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(Units.degreesToRadians(3));

        configureBindings();
    }

    public PhotonCamera getCammy(){
        return visionSystem.getCamera();
    }

    public Vision getVisionSubsystem(){
        return visionSystem;
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed + xForwardAdjustment * MaxSpeed) 
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed + yForwardAdjustment * MaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate + rotationalAdjustment ) 
            ).alongWith(
                Commands.run(()->{
                    if (Robot.isSimulation()){
                        drivetrain.getSimulatedDrivetrain().runChassisSpeeds(
                            new ChassisSpeeds(  -joystick.getLeftY() * MaxSpeed + xForwardAdjustment * MaxSpeed,
                                                -joystick.getLeftX() * MaxSpeed + yForwardAdjustment * MaxSpeed, 
                                                -joystick.getRightX() * MaxAngularRate + rotationalAdjustment),
                            Translation2d.kZero,
                            true,
                            false);
                    }
                })
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));
        joystick.y().onTrue(
            Commands.runOnce(()->{
                if (Robot.isSimulation()){
                    Arena2025Reefscape.getInstance().addGamePiece(new ReefscapeCoralOnField(new Pose2d(2, 2, Rotation2d.fromDegrees(90))));
                }
            })
        );

        joystick.rightBumper().onTrue(Commands.runOnce(
        ()->{
            rotationController.reset(drivetrain.getRotation3d().getZ());
        }    
        ));

        joystick.rightBumper().whileTrue(Commands.run(
            ()->{
                Optional<PhotonTrackedTarget> trackedTarget = visionSystem.getBestTargetPose();
                
                if (trackedTarget.isPresent() && trackedTarget.get().fiducialId == targetId) {
                    
                    rotationController.setGoal(0);
                    rotationalAdjustment = rotationController.calculate(Units.degreesToRadians(trackedTarget.get().getYaw()));

                    DogLog.log("rotationalAdjustment", rotationalAdjustment);
                    DogLog.log("targetyaw", trackedTarget.get().getYaw());
                    DogLog.log("targetpitch", trackedTarget.get().getPitch());
                    
                    DogLog.log("targetskew", trackedTarget.get().getSkew());

                } else {
                    rotationalAdjustment = 0;
                }


                if (targetId != -1){
                    
                    xForwardAdjustment = -1.0 * VisionConstants.kTagLayout.getTagPose(targetId).get().getTranslation().toTranslation2d().minus(drivetrain.getPose().getTranslation()).getMeasureX().in(Meters);
                    yForwardAdjustment = -1.0 * VisionConstants.kTagLayout.getTagPose(targetId).get().getTranslation().toTranslation2d().minus(drivetrain.getPose().getTranslation()).getMeasureY().in(Meters);
                    
                } else {
                    trackedTarget = visionSystem.getBestTargetPose();
                    if (trackedTarget.isPresent()){
                        targetId = trackedTarget.get().fiducialId;
                    }

                    DogLog.log("targetID", targetId);
                }
                
                
            }
        ));

        joystick.rightBumper().onFalse(Commands.runOnce(()->{
            targetId = -1;
            rotationalAdjustment = 0;
            xForwardAdjustment = 0;
            yForwardAdjustment = 0;

            DogLog.log("targetID", targetId);
            DogLog.log("rotationalAdjustment", rotationalAdjustment);
        }));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));



        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void setFieldRelativeFromCamera(){
        Optional<Pose3d> fieldPose = visionSystem.getFieldPose();
        if (fieldPose.isPresent()){
            DogLog.log("Vision/visionPose", fieldPose.get().toPose2d());
            drivetrain.addVisionMeasurement(fieldPose.get().toPose2d(), Timer.getTimestamp());
            if (Robot.isSimulation()){
                drivetrain.getSimulatedDrivetrain().addVisionEstimation(fieldPose.get().toPose2d(), Timer.getTimestamp());
            }
        }
        
    }

    public AutoRoutine myAuto() {
        AutoRoutine routine = autoFactory.newRoutine("myRoutine");

        AutoTrajectory spinInCircle = routine.trajectory("moveInSquare");

        routine.active().onTrue(
            Commands.sequence(
                spinInCircle.resetOdometry(),
                spinInCircle.cmd()
            )
            
        );

        return routine;
    }

    public Command getAutonomousCommand() {
        return myAuto().cmd();
    }

}