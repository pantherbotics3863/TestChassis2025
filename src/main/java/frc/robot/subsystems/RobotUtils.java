// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.GeneralConstants;
import frc.robot.Robot;

/** Add your docs here. */
public class RobotUtils {

    // Adjustments singleton to throw stuff at.
    private static Adjustments2d adjustments = new Adjustments2d();
    private static final double kToleranceMeters = 0.1;
    private static final double kToleranceRadians = 0.1;

    public static final FieldCentric kGeneralFieldCentric = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
    .withDeadband(GeneralConstants.kMaxSpeed * 0.1);


    public static class Adjustments2d {
        private Distance xAdjustment;
        private Distance yAdjustment;
        private Angle rotationAdjustment;

        public Adjustments2d(double xAdjustment, double yAdjustment, double rotationAdjustment){
            this.xAdjustment = Meters.of(xAdjustment);
            this.yAdjustment = Meters.of(yAdjustment);
            this.rotationAdjustment = Radians.of(rotationAdjustment);
        }

        public Adjustments2d(){
            this.xAdjustment = Meters.zero();
            this.yAdjustment = Meters.zero();
            this.rotationAdjustment = Radians.zero();
        }

        // In SI units
        public double getXAdjustment(){
            return xAdjustment.in(Meters);
        }

        public double getYAdjustment(){
            return yAdjustment.in(Meters);
        }

        public double getRotationAdjustment(){
            return rotationAdjustment.in(Radians);
        }

        // In WPILib Measurements
        public Distance getXAdjustmentMeasure(){
            return xAdjustment;
        }

        public Distance getYAdjustmentMeasure(){
            return yAdjustment;
        }

        public Angle getRotationAdjustmentMeasure(){
            return rotationAdjustment;
        }

        // Set commands in SI

        public void setXAdjustment(double xAdjustment){
            this.xAdjustment = Meters.of(xAdjustment);
        }

        public void setYAdjustment(double yAdjustment){
            this.yAdjustment = Meters.of(yAdjustment);
        }

        public void setRotationAdjustment(double rotationAdjustment){
            this.rotationAdjustment = Radians.of(rotationAdjustment);
        }

        // Set commands in WPILib Measurements
        public void setXAdjustmentMeasure(Distance xAdjustment){
            this.xAdjustment = xAdjustment;
        }

        public void setYAdjustmentMeasure(Distance yAdjustment){
            this.yAdjustment = yAdjustment;
        }

        public void setRotationAdjustmentMeasure(Angle rotationAdjustment){
            this.rotationAdjustment = rotationAdjustment;
        }
    }

    // Consider getting rid of Adjustments2d and using Transform2d
    
    public static Adjustments2d getAdjustmentToPose(Pose2d goalPose, Pose2d currentPose) {
        Adjustments2d adjustments = new Adjustments2d();
        Translation2d differenceTranslation = goalPose.getTranslation().minus(currentPose.getTranslation());
        Rotation2d differenceRotation = goalPose.getRotation().minus(currentPose.getRotation());
        adjustments.setXAdjustmentMeasure(differenceTranslation.getMeasureX());
        adjustments.setYAdjustmentMeasure(differenceTranslation.getMeasureY());
        adjustments.setRotationAdjustmentMeasure(differenceRotation.getMeasure());
        return adjustments;
    }

    public static boolean isWithinTolerance(Pose2d poseOne, Pose2d poseTwo, double toleranceMeters, double toleranceRadians) {
        Transform2d difference = poseOne.minus(poseTwo);
        return difference.getTranslation().getNorm() <= toleranceMeters && Math.abs(difference.getRotation().getRadians()) <= toleranceRadians;
    }

    public static Command moveToPose(CommandSwerveDrivetrain drivetrain, Pose2d goalPose) {
        return Commands.run(()->{
            adjustments = getAdjustmentToPose(goalPose, drivetrain.getPose());
            drivetrain.applyRequest( ()->
                kGeneralFieldCentric
                .withVelocityX(adjustments.getXAdjustment())
                .withVelocityY(adjustments.getYAdjustment())
                .withRotationalRate(adjustments.getRotationAdjustment())
            );

            if (Robot.isSimulation()) {
                drivetrain.getSimulatedDrivetrain().runChassisSpeeds(
                    new ChassisSpeeds(
                        adjustments.getXAdjustment(), 
                        adjustments.getYAdjustment(), 
                        adjustments.getRotationAdjustment()
                    ), 
                    Translation2d.kZero,
                    true,
                    false);
            }
            DogLog.log("moveToPose/xAdjustment", adjustments.getXAdjustment());
            DogLog.log("moveToPose/yAdjustment", adjustments.getYAdjustment());
            DogLog.log("moveToPose/rotationAdjustment", adjustments.getRotationAdjustment());

        })
        .until(()->isWithinTolerance(drivetrain.getPose(), goalPose, kToleranceMeters, kToleranceRadians))
        ;
    }
}
