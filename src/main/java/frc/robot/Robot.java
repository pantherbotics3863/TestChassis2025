// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Vision.VisionSim;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final VisionSim visionSim;

  public Robot() {
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();
    
    m_robotContainer = new RobotContainer();
    visionSim = new VisionSim(m_robotContainer.getCammy());
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
    m_robotContainer.setFieldRelativeFromCamera();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationInit(){
    Arena2025Reefscape.getInstance().addDriveTrainSimulation(m_robotContainer.drivetrain.getSimulatedDrivetrain().getDriveTrainSimulation());
  }

  @Override
  public void simulationPeriodic() {
    visionSim.visionPeriodic(m_robotContainer.drivetrain.getSimPose());
    Arena2025Reefscape.getInstance().simulationPeriodic();

    Logger.recordOutput("FieldSimulation/Coral", 
    Arena2025Reefscape.getInstance().getGamePiecesArrayByType("Coral"));
    m_robotContainer.drivetrain.getSimulatedDrivetrain().periodic();

    Logger.recordOutput("FieldSimulation/RobotPose", m_robotContainer.drivetrain.getSimPose());
    Logger.recordOutput("FieldSimulation/RobotOdometry", m_robotContainer.drivetrain.getSimOdometryPose());

    Optional<PhotonTrackedTarget> trackedTarget = m_robotContainer.getVisionSubsystem().getBestTargetPose();
    
    if (trackedTarget.isPresent()){
      Logger.recordOutput("Vision/TargetData/ID", trackedTarget.get().fiducialId);
      Logger.recordOutput("Vision/TargetData/Yaw", trackedTarget.get().getYaw());
      Logger.recordOutput("Vision/TargetData/Yaw_RAD", trackedTarget.get().getYaw() * Math.PI /180.0);
    }

  }
}
