// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.ironmaple.simulation.seasonspecific.reefscape2025.Arena2025Reefscape;
import org.photonvision.targeting.PhotonTrackedTarget;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Vision.VisionSim;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final VisionSim visionSim;

  public Robot() {
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
    m_robotContainer.drivetrain.getSimulatedDrivetrain().setSimulationWorldPose(GeneralConstants.initialPose);
  }

  @Override
  public void simulationPeriodic() {
    visionSim.visionPeriodic(m_robotContainer.drivetrain.getSimPose());
    Arena2025Reefscape.getInstance().simulationPeriodic();
    
    DogLog.log("FieldSimulation/Coral", 
    Arena2025Reefscape.getInstance().getGamePiecesArrayByType("Coral"));
    m_robotContainer.drivetrain.getSimulatedDrivetrain().periodic();

    DogLog.log("FieldSimulation/RobotPose", m_robotContainer.drivetrain.getSimPose());
    DogLog.log("FieldSimulation/RobotOdometry", m_robotContainer.drivetrain.getSimOdometryPose());

    Optional<PhotonTrackedTarget> trackedTarget = m_robotContainer.getVisionSubsystem().getBestTargetPose();
    
    if (trackedTarget.isPresent()){
      DogLog.log("Vision/TargetData/ID", trackedTarget.get().fiducialId);
      DogLog.log("Vision/TargetData/Yaw", trackedTarget.get().getYaw());
      DogLog.log("Vision/TargetData/Yaw_RAD", trackedTarget.get().getYaw() * Math.PI /180.0);
    }

  }
}
