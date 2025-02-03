package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PutterSubsystems;
import java.util.Optional;

import choreo.Choreo;
import choreo.trajectory.DifferentialSample;
import choreo.trajectory.Trajectory;

public class RobotContainer {
    private final Driver driver = new Driver();

    public final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final PutterSubsystems putterSubsytems = new PutterSubsystems();

    private final Timer timer = new Timer();

    private final Optional<Trajectory<DifferentialSample>> trajectory = Choreo.loadTrajectory("kop"); 

    private final DriveCmd driveJoystickCmd = new DriveCmd(driveSubsystem, driver);

    public RobotContainer() {
        this.driveSubsystem.setDefaultCommand(this.driveJoystickCmd);
        this.configBindings();
    }

    public void configBindings() {
        this.driver.Putter()
                .whileTrue(this.putterSubsytems.cmdExecute());

        this.driver.PutterCorrection()
                .whileTrue(this.putterSubsytems.cmdExecuteCorrection());

    }

    public Command getAutonomousCommand() {
      return null;
}
// return new SequentialCommandGroup(
// new ParallelRaceGroup(
// Commands.runEnd(()->this.driveSubsystem.execute(5, 5),
// this.driveSubsystem::stopModules, this.driveSubsystem),
// new WaitCommand(1)
// ),
// new ParallelRaceGroup(
// Commands.runEnd(()->this.driveSubsystem.execute(-1, -1),
// this.driveSubsystem::stopModules, this.driveSubsystem),
// new WaitCommand(0.2)
// ),
// new ParallelRaceGroup(
// Commands.runEnd(this.putterSubsytems::execute,this.putterSubsytems::stop,this.putterSubsytems),
// new WaitCommand(0.3)
// ),
// new ParallelRaceGroup(
// Commands.runEnd(()->this.driveSubsystem.execute(12, 48),
// this.driveSubsystem::stopModules, this.driveSubsystem),
// new WaitCommand(1.5)
// ),
// new ParallelRaceGroup(
// Commands.runEnd(()->this.driveSubsystem.execute(-5,-5),
// this.driveSubsystem::stopModules, this.driveSubsystem),
// new WaitCommand(0.5)
// ),
// new ParallelRaceGroup(
// Commands.runEnd(()->this.driveSubsystem.execute(-5, -5),
// this.driveSubsystem::stopModules, this.driveSubsystem),
// new WaitCommand(1)
// ),
// new ParallelRaceGroup(
// Commands.runEnd(()->this.driveSubsystem.execute(1, 1),
// this.driveSubsystem::stopModules, this.driveSubsystem),
// new WaitCommand(0.2)
// ),
// new ParallelRaceGroup(
// Commands.runEnd(this.putterSubsytems::execute,this.putterSubsytems::stop,this.putterSubsytems),
// new WaitCommand(2)
// )
// );
}
