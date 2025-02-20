package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.DeviceId;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class DriveModule {
    private final SparkMax motor;
    private final SparkMax followmotor;
    private final RelativeEncoder encoder;

    public DriveModule(int port, boolean reverse, int followport, boolean followreverse) {
        this.motor = new SparkMax(port, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        config
                .inverted(reverse)
                .idleMode(IdleMode.kBrake);
        this.motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        this.encoder = this.motor.getEncoder();

        this.followmotor = new SparkMax(followport, MotorType.kBrushless);
        SparkMaxConfig followConfig = new SparkMaxConfig();
        followConfig
                .inverted(followreverse)
                .idleMode(IdleMode.kBrake)
                .follow(motor);
        this.followmotor.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void stop() {
        this.motor.stopMotor();
    }

    public double getPosition() {
        return this.encoder.getPosition();
    }

    public double getVelocity() {
        return this.encoder.getVelocity();
    }
    public void setVoltage(double speed) {
        this.motor.setVoltage(speed);
    }
    public void set(double speed) {
        this.motor.set(speed);
    }
}
