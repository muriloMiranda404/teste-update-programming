package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.FRC9485.vision.LimelightHelpers;
import frc.robot.Constants.Components;
import frc.robot.Constants.vision;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private Pigeon2 pigeon2;

  public Robot(){
    pigeon2 = new Pigeon2(Components.PIGEON);
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    pigeon2.reset();

    LimelightHelpers.SetFiducialIDFiltersOverride("", vision.ALL_TAGS);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {;
    
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    System.out.println("Alliance: " + DriverStation.getAlliance());
    System.out.println("Auto mirror active: " + (
    DriverStation.getAlliance().isPresent() &&
    DriverStation.getAlliance().get() == DriverStation.Alliance.Red
    ));
    System.out.println("Starting pose: " + SwerveSubsystem.getInstance().getPose());

  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {
  }
}