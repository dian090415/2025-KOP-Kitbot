package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeArmSubsystems;

public class IntakeArmCmd extends Command {
	private final IntakeArmSubsystems intakeArmSubsystem;
	private final XboxController controller;

	public IntakeArmCmd(IntakeArmSubsystems intakeArmSubsystem , XboxController controller){
		this.intakeArmSubsystem = intakeArmSubsystem;
		this.controller = controller;
		this.addRequirements(this.intakeArmSubsystem);
	}

	@Override
	public void initialize() {
	}

	@Override
	public void execute() {
		double speed = MathUtil.applyDeadband(this.controller.getLeftY(), Constants.Drive.DEAD_BAND) * 0.2;
		this.intakeArmSubsystem.execute(speed);
	}

	@Override
	public void end(boolean interrupted) {
		this.intakeArmSubsystem.stop();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}