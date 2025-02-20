package frc.robot.subsystems;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import choreo.trajectory.DifferentialSample;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.Constants.MotorReverse;
import frc.robot.DeviceId.DriveMotor;

public class DriveSubsystem extends SubsystemBase {
    private final DriveModule Left = new DriveModule(DriveMotor.FRONT_LEFT, MotorReverse.FRONT_LEFT,
            DriveMotor.BACK_LEFT, MotorReverse.BACK_LEFT);
    private final DriveModule Right = new DriveModule(DriveMotor.FRONT_RIGHT, MotorReverse.FRONT_RIGHT,
            DriveMotor.BACK_RIGHT, MotorReverse.BACK_RIGHT);

    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    // 機器人運動學
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.metersToInches(0.568)); // 設定左右輪間距

    // 機器人位置估算器
    private final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(kinematics,
            this.gyro.getRotation2d(), getLeftSpeed(), getRightSpeed(), new Pose2d());

    private final PIDController xController = new PIDController(10, 0.0, 0.0);
    private final PIDController yController = new PIDController(10, 0.0, 0.0);
    private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

    private final PIDController leftPIDController = new PIDController(0.01, 0.0, 0.0);
    private final PIDController rightPIDController = new PIDController(0.01, 0.0, 0.0);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1.0, 0.5, 0.1); // 前饋常數

    private double timeStates;
    private double nowTime;
    private final double KOP_WHITE_WHEEL = 0.0762;

    public DriveSubsystem() {
        this.headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void resetPose(Pose2d pose) {
        this.poseEstimator.resetPosition(gyro.getRotation2d(), getLeftSpeed(), getRightSpeed(), pose);
    }

    public void resetOdometry(Pose2d pose) {
        this.poseEstimator.resetPosition(pose.getRotation(), getLeftVelocity(), getRightVelocity(), pose);
    }

    public Pose2d getPose() {
        SmartDashboard.putString("pose", this.poseEstimator.getEstimatedPosition().toString());
        return this.poseEstimator.getEstimatedPosition();
    }

    public void followTrajectory(DifferentialSample sample) {
        // 獲取當前機器人位置
        Pose2d pose = getPose();
    
        // 計算左右輪速度
        double leftSpeed = sample.vl + xController.calculate(pose.getX(), sample.x);
        double rightSpeed = sample.vr + headingController.calculate(pose.getRotation().getRadians(), sample.heading);
    
        // 設定左右輪速度
        autoexecute(leftSpeed, rightSpeed);
    }

    // public void driveFieldRelative(ChassisSpeeds speeds) {
    //     // 使用運動學模型來計算左右輪的速度
    //     var wheelSpeeds = kinematics.toWheelSpeeds(speeds);

    //     double leftSpeed = wheelSpeeds.leftMetersPerSecond;
    //     double rightSpeed = wheelSpeeds.rightMetersPerSecond;

    //     // 呼叫馬達控制函式，將計算出來的左右輪速度轉換為馬達輸入
    //     autoexecute(leftSpeed, rightSpeed);
    // }

    public void updatePose() {
        this.poseEstimator.update(this.gyro.getRotation2d(), this.getLeftVelocity(), this.getRightVelocity());
    }

    public void updateTime() {
        nowTime = Timer.getFPGATimestamp() - timeStates;
        this.timeStates = Timer.getFPGATimestamp();
    }

    public double getLeftSpeed() {
        return this.Left.getVelocity() / 60 * (1 / 10.71) * (KOP_WHITE_WHEEL * Math.PI * 2) * nowTime;
    }

    public double getRightSpeed() {
        return this.Right.getVelocity() / 60 * (1 / 10.71) * (KOP_WHITE_WHEEL * Math.PI * 2) * nowTime;
    }

    public double getLeftVelocity() {
        return this.Left.getVelocity();
    }

    public double getRightVelocity() {
        return this.Right.getVelocity();
    }

    public double getRightWheelPosition() {
        return this.Right.getPosition() / 13.5 * KOP_WHITE_WHEEL;
    }

    public double getLeftWheelPosition() {
        return this.Left.getPosition() / 13.5 * KOP_WHITE_WHEEL;
    }

    public void stopModules() {
        this.Left.stop();
        this.Right.stop();
    }

    public void execute(double leftSetpoint, double rightSetpoint) {
        // 獲取當前速度（以距離的變化率計算）
        double leftVelocity = this.getLeftSpeed();
        double rightVelocity = this.getRightSpeed();

        // PID 輸出
        double leftPIDOutput = this.leftPIDController.calculate(leftVelocity, leftSetpoint);
        double rightPIDOutput = this.rightPIDController.calculate(rightVelocity, rightSetpoint);

        // 前饋補償
        double leftFeedforward = this.feedforward.calculate(leftSetpoint);
        double rightFeedforward = this.feedforward.calculate(rightSetpoint);

        // 合成電壓輸出
        double leftVoltage = leftPIDOutput + leftFeedforward;
        double rightVoltage = rightPIDOutput + rightFeedforward;

        // 限制電壓範圍 (-12V 到 12V)
        leftVoltage = MathUtil.clamp(leftVoltage, -12.0, 12.0);
        rightVoltage = MathUtil.clamp(rightVoltage, -12.0, 12.0);

        // 設定馬達電壓
        this.Left.setVoltage(leftVoltage);
        this.Right.setVoltage(rightVoltage);

        this.getPose();
        SmartDashboard.putNumber("leftVelocity", leftVelocity);
        SmartDashboard.putNumber("rightVelocityy", rightVelocity);
        SmartDashboard.putNumber("leftSetpoint", leftSetpoint);
        SmartDashboard.putNumber("rightSetpoin", rightSetpoint);
        SmartDashboard.putNumber("RightWheelPosition", this.Right.getVelocity());
        SmartDashboard.putNumber("LeftWheelPositio", this.Left.getVelocity());
    }

    public void autoexecute(double leftSetpoint, double rightSetpoint) {

        double leftFeedforward = this.feedforward.calculate(leftSetpoint);
        double rightFeedforward = this.feedforward.calculate(rightSetpoint);

        double leftVoltage = MathUtil.clamp(leftFeedforward, -6.0, 6.0);
        double rightVoltage = MathUtil.clamp(rightFeedforward, -6.0, 6.0);

        this.Left.setVoltage(-leftVoltage);
        this.Right.setVoltage(-rightVoltage);

        SmartDashboard.putNumber("autoleftspeed", leftVoltage);
        SmartDashboard.putNumber("autorightSpeed", rightVoltage);
    }

    @Override
    public void periodic() {
        this.updatePose();
        this.updateTime();
        this.poseEstimator.update(this.gyro.getRotation2d(),
                new DifferentialDriveWheelPositions(this.getLeftWheelPosition(), this.getRightWheelPosition()));
    }
}