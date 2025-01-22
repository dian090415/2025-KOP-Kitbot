
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Drive;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCmd extends Command {
    private final DriveSubsystem driveSubsystem;
    private final XboxController controller;

    public DriveCmd(DriveSubsystem driveSubsystem, XboxController controller) {
        this.driveSubsystem = driveSubsystem;
        this.controller = controller;
        this.addRequirements(this.driveSubsystem);
    }

    @Override
    public void execute() {
        double driveSpeed = MathUtil.applyDeadband(this.controller.getLeftY(), Drive.DEAD_BAND) * Drive.MAX_SPEED;
        double turnSpeed = MathUtil.applyDeadband(this.controller.getRightX(), Drive.DEAD_BAND) * Drive.MAX_TURN_SPEED;
    
        double leftSpeed = driveSpeed - turnSpeed;
        double rightSpeed = driveSpeed + turnSpeed;
    
        this.driveSubsystem.execute(leftSpeed, rightSpeed);
        SmartDashboard.putNumber("LeftSpeed", leftSpeed);
        SmartDashboard.putNumber("RightSpeed", rightSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        this.driveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
