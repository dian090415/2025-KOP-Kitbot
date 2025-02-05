package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.DeviceId;

public class IntakeArmSubsystems extends SubsystemBase {
    private final SparkMax motor;

    private final DutyCycleEncoder encoder = new DutyCycleEncoder(0);
    private final PIDController lifterPid = new PIDController(0.01, 0, 0); // TODO

    private final double MIN_DEGREE = -0.388300284707507;
    private final double MAX_DEGREE = 0.368803809220095;

    public IntakeArmSubsystems() {
        this.motor = new SparkMax(DeviceId.controller.PutterSubsytems, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config
                .inverted(true)
                .idleMode(IdleMode.kBrake);
        this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        SmartDashboard.getNumber("intakearm encoder", this.encoder.get());
    }

    public void execute(double speed) {
        if (this.encoder.get() >= this.MIN_DEGREE && this.encoder.get() <= this.MAX_DEGREE) {
            this.motor.set(speed);
        } else if (this.encoder.get() > this.MAX_DEGREE && speed >= 0.0) {
            this.motor.set(speed);
        } else if (this.encoder.get() < this.MIN_DEGREE && speed <= 0.0) {
            this.motor.set(speed);
        } else {
            this.motor.set(0);
        }
    }

    public void stop() {
        this.motor.stopMotor();
    }

    public void armto(double angle) {
        double speed = MathUtil.applyDeadband(this.lifterPid.calculate(angle, this.encoder.get()),
                Constants.Drive.DEAD_BAND);
        this.execute(speed);
    }

    public Command isUp() {
        return new WaitUntilCommand(() -> this.encoder.get() <= this.MIN_DEGREE + 0.05); // 判斷式
    }

    public Command isDown() {
        return new WaitUntilCommand(() -> this.encoder.get() >= this.MAX_DEGREE + 0.05);
    }

    public Command autoUp() {
        return new ParallelDeadlineGroup(
                this.isUp(),
                Commands.runEnd(() -> this.armto(MAX_DEGREE), this::stop, this));
    }

    public Command autodown() {
        return new ParallelDeadlineGroup(
                this.isDown(),
                Commands.runEnd(() -> this.armto(MIN_DEGREE), this::stop, this));
    }
}
