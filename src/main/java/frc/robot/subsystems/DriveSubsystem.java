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
import frc.robot.Constants.MotorReverse;
import frc.robot.DeviceId.DriveMotor;

public class DriveSubsystem extends SubsystemBase {
    private final DriveModule Left = new DriveModule(DriveMotor.FRONT_LEFT, MotorReverse.FRONT_LEFT, DriveMotor.BACK_LEFT, MotorReverse.BACK_LEFT);
    private final DriveModule Right = new DriveModule(DriveMotor.FRONT_RIGHT, MotorReverse.FRONT_RIGHT, DriveMotor.BACK_RIGHT, MotorReverse.BACK_RIGHT);

    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    // 機器人運動學
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.metersToInches(56.8)); // 設定左右輪間距

    // 機器人位置估算器
    private final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(kinematics,
            this.gyro.getRotation2d(), getLeftSpeed(), getRightSpeed(), new Pose2d());

    private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(7.5, 0.0, 0.0);

    private final PIDController leftPIDController = new PIDController(0.01, 0.0, 0.0);
    private final PIDController rightPIDController = new PIDController(0.01, 0.0, 0.0);
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(1.0, 0.5, 0.1); // 前饋常數

    private double timeStates;
    private double nowTime;
    private final double KOP_WHITE_WHEEL = 0.0762;

    public DriveSubsystem() {
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(gyro.getRotation2d(), getLeftSpeed(), getRightSpeed(), pose);
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(pose.getRotation(), getLeftVelocity(), getRightVelocity(), pose);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void driveFieldRelative(ChassisSpeeds speeds) {
        // 使用 Gyro 來轉換速度，使機器人能夠場地相對移動
        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond, // 坦克驅動不會用到 Y 軸速度
                speeds.omegaRadiansPerSecond,
                gyro.getRotation2d());

        // 傳給下一層處理
        driveRobotRelative(robotRelativeSpeeds);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        // 透過 Kinematics 計算左右輪速率
        var wheelSpeeds = kinematics.toWheelSpeeds(speeds);

        double leftSpeed = wheelSpeeds.leftMetersPerSecond;
        double rightSpeed = wheelSpeeds.rightMetersPerSecond;

        // 呼叫馬達控制函式
        execute(leftSpeed, rightSpeed);
    }

    public void updatePose() {
        this.poseEstimator.update(this.gyro.getRotation2d(), this.getLeftVelocity(), this.getRightVelocity());
    }

    public void followTrajectory(DifferentialSample sample) {
        // 取得當前機器人位置
        Pose2d pose = getPose();

        // 使用 PID 控制器計算速度
        double velocity = sample.omega + xController.calculate(pose.getX(), sample.x);
        double omega = sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading);

        // KOP 不支援 Y 軸移動，所以只設定 X 速度和角速度
        ChassisSpeeds speeds = new ChassisSpeeds(velocity, 0.0, omega);

        driveRobotRelative(speeds);
    }

    public void updateTime() {
        nowTime = Timer.getFPGATimestamp() - timeStates;
        this.timeStates = Timer.getFPGATimestamp();
    }

    public double getLeftSpeed() {
        return this.Left.getVelocity() / 60 / 13.5 * KOP_WHITE_WHEEL * 2 * Math.PI * nowTime;
    }

    public double getRightSpeed() {
        return this.Right.getVelocity() / 60 / 13.5 * KOP_WHITE_WHEEL * 2 * Math.PI * nowTime;
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
        double leftPIDOutput = leftPIDController.calculate(leftVelocity, leftSetpoint);
        double rightPIDOutput = rightPIDController.calculate(rightVelocity, rightSetpoint);

        // 前饋補償
        double leftFeedforward = feedforward.calculate(leftSetpoint);
        double rightFeedforward = feedforward.calculate(rightSetpoint);

        // 合成電壓輸出
        double leftVoltage = leftPIDOutput + leftFeedforward;
        double rightVoltage = rightPIDOutput + rightFeedforward;

        // 限制電壓範圍 (-12V 到 12V)
        leftVoltage = MathUtil.clamp(leftVoltage, -12.0, 12.0);
        rightVoltage = MathUtil.clamp(rightVoltage, -12.0, 12.0);

        // 設定馬達電壓
        this.Left.setVoltage(leftVoltage);
        this.Right.setVoltage(rightVoltage);

        SmartDashboard.putNumber("leftVelocity", leftVelocity);
        SmartDashboard.putNumber("rightVelocityy", rightVelocity);
        SmartDashboard.putNumber("leftSetpoint", leftSetpoint);
        SmartDashboard.putNumber("rightSetpoin", rightSetpoint);
    }

    @Override
    public void periodic() {
        this.updatePose();
        this.updateTime();
        this.poseEstimator.update(this.gyro.getRotation2d(), new DifferentialDriveWheelPositions(this.getLeftWheelPosition(), this.getRightWheelPosition()));
    }
}