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
import frc.robot.Controller;
import frc.robot.commands.DriveCmd;
import frc.robot.commands.IntakeArmCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeArmSubsystems;
import frc.robot.subsystems.IntakeSubsystems;
import frc.robot.subsystems.PutterSubsystems;

import java.util.Optional;

import choreo.auto.AutoFactory;
import choreo.Choreo;
import choreo.trajectory.DifferentialSample;
import choreo.trajectory.Trajectory;

public class RobotContainer {

        private final Driver driver = new Driver();
        private final Controller controller = new Controller();

        public final DriveSubsystem driveSubsystem = new DriveSubsystem();
        private final PutterSubsystems putterSubsytems = new PutterSubsystems();
        private final IntakeArmSubsystems intakeArmSubsystems = new IntakeArmSubsystems();
        private final IntakeSubsystems intakeSubsystems = new IntakeSubsystems();

        private final Optional<Trajectory<DifferentialSample>> trajectory = Choreo.loadTrajectory("kop");

        private final DriveCmd driveJoystickCmd = new DriveCmd(driveSubsystem, controller);
        private final IntakeArmCmd IntakeArmCmd = new IntakeArmCmd(intakeArmSubsystems, controller);

        public RobotContainer() {
                this.driveSubsystem.setDefaultCommand(this.driveJoystickCmd);
                this.intakeArmSubsystems.setDefaultCommand(this.IntakeArmCmd);
                this.configBindings();
        }

        public void configBindings() {
                this.controller.Putter()
                                .whileTrue(this.putterSubsytems.cmdExecute());

                this.controller.PutterCorrection()
                                .whileTrue(this.putterSubsytems.cmdExecuteCorrection());

                this.controller.IntakelifeUp()
                                .onTrue(this.intakeArmSubsystems.down()
                                                .alongWith(this.intakeSubsystems.Cmdbackexecute()))
                                .onFalse(this.intakeArmSubsystems.Up());
                this.controller.IntakeLifeDown()
                                .onTrue(this.intakeArmSubsystems.down()
                                                .alongWith(this.intakeSubsystems.Cmdexecute()))
                                .onFalse(this.intakeArmSubsystems.keep());
                this.controller.getCoral()
                                .onTrue(this.intakeArmSubsystems.coral()
                                .alongWith(this.intakeSubsystems.Cmdexecute()))
                                .onFalse(this.intakeArmSubsystems.Up());
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
