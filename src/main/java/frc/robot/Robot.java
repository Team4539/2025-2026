// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.Elastic;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();

  
  }
  public void Robotinit() {
    
    }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    Elastic.selectTab("Teleop");
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {
    WebServer.stop(5800);
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // Shuffleboard.startRecording();
    Elastic.selectTab("Autonomous");
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    Shuffleboard.stopRecording();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // Shuffleboard.startRecording();
    Elastic.selectTab("Teleop");
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    Shuffleboard.stopRecording();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    //Elastic.selectTab("DEBUG");
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
