package frc.robot;

import java.util.Optional;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.trajectory.DifferentialSample;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private final Optional<Trajectory<DifferentialSample>> trajectory = Choreo.loadTrajectory("kop");
  // private final AutoFactory autoFactory;

  private final Timer timer = new Timer();
  // public Robot(){
  //   autoFactory = new AutoFactory(
  //     this.m_robotContainer.driveSubsystem::getPose, // A function that returns the current robot pose
  //     this.m_robotContainer.driveSubsystem::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
  //     this.m_robotContainer.driveSubsystem::followTrajectory, // The drive subsystem trajectory follower 
  //     true, // If alliance flipping should be enabled 
  //     this.m_robotContainer.driveSubsystem // The drive subsystem
  // );
  // }

  @Override
  public void robotInit() {

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

     if (trajectory.isPresent()) {
            // Get the initial pose of the trajectory
            Optional<Pose2d> initialPose = trajectory.get().getInitialPose(isRedAlliance());

            if (initialPose.isPresent()) {
                // Reset odometry to the start of the trajectory
                this.m_robotContainer.driveSubsystem.resetOdometry(initialPose.get());
            }
        }

        // Reset and start the timer when the autonomous period begins
        timer.restart();
  }

  @Override
  public void autonomousPeriodic() {
     if (trajectory.isPresent()) {
            // Sample the trajectory at the current time into the autonomous period
            Optional<DifferentialSample> sample = trajectory.get().sampleAt(timer.get(), isRedAlliance());

            if (sample.isPresent()) {
              this.m_robotContainer.driveSubsystem.followTrajectory(sample.get());
            }
        }
    }

    private boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
    }

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
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}