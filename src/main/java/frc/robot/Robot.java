package frc.robot;

import static edu.wpi.first.units.Units.Seconds;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnField;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.FRC9485.constants.AlingConstants;
import frc.FRC9485.vision.LimelightHelpers;
import frc.robot.GeralConstants.Components;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private Pigeon2 pigeon2;

  private SimulatedArena simulatedArena = SimulatedArena.getInstance();

  public Robot(){
    SimulatedArena.overrideInstance(simulatedArena);

    pigeon2 = new Pigeon2(Components.PIGEON);
  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    if(DriverStation.getAlliance().get() == Alliance.Blue) pigeon2.reset();

    LimelightHelpers.SetFiducialIDFiltersOverride("", AlingConstants.ALL_TAGS);
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
    simulatedArena.simulationPeriodic();
    SimulatedArena.overrideSimulationTimings(Time.ofRelativeUnits(0.01, Seconds), 3);

    simulatedArena.addGamePiece(new ReefscapeCoralOnField(new Pose2d(3, 3, new Rotation2d(0))));
  }
}