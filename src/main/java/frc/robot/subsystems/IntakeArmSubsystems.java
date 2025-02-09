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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.DeviceId;

public class IntakeArmSubsystems extends SubsystemBase {
    private final SparkMax motor;

    private final DutyCycleEncoder encoder = new DutyCycleEncoder(0);
    private final PIDController lifterPid = new PIDController(1.15, 0, 0); // TODO

    private final double MIN_DEGREE = 0.251735706293393;
    private final double MAX_DEGREE = 0.06147320153683;
    private final double Keep = 0.229223355730584;

    public IntakeArmSubsystems() {
        this.motor = new SparkMax(DeviceId.controller.intakearmSubsytems, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config
                .inverted(false)
                .idleMode(IdleMode.kBrake);
        this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void execute(double speed) {
        if (this.encoder.get() >= this.MIN_DEGREE && this.encoder.get() <= this.MAX_DEGREE) {
            this.motor.set(speed);
        } else if (this.encoder.get() > this.MAX_DEGREE && speed <= 0.0) {
            this.motor.set(speed);
        } else if (this.encoder.get() < this.MIN_DEGREE && speed >= 0.0) {
            this.motor.set(speed);
        } else {
            this.motor.set(0);
        }
        SmartDashboard.putNumber("intakearm encoder", this.encoder.get());
        SmartDashboard.putNumber("intakearm speed", speed);
    }

    public void stop() {
        this.motor.stopMotor();
    }

    public void armto(double angle) {
        double speed = MathUtil.applyDeadband(this.lifterPid.calculate(this.encoder.get(), angle),
                0.01);
        this.motor.set(speed);
        SmartDashboard.putNumber("intakearm autospeed", speed);
        SmartDashboard.putNumber("intakearm encoder", this.encoder.get());
    }

    public Command Up() {
        return Commands.runEnd(() -> this.armto(MAX_DEGREE), this::stop, this);
    }

    public Command down() {
        return Commands.runEnd(() -> this.armto(MIN_DEGREE), this::stop, this);
    }
    public Command keep() {
        return Commands.runEnd(() -> this.armto(Keep), this::stop, this);
}
}