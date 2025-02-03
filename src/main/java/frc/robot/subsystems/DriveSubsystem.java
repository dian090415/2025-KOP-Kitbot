package frc.robot.subsystems;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorReverse;
import frc.robot.DeviceId.DriveMotor;

public class DriveSubsystem extends SubsystemBase {
    private final DriveModule Left;
    private final DriveModule Right;


    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    // 機器人運動學
private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(20)); // 設定左右輪間距

// 機器人位置估算器
private final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(
    kinematics,
    gyro.getRotation2d(),        // 起始角度 (來自 NavX)
    this.leftEncoder(),         // 左編碼器位置 (初始值)
    rightEncoderPosition,        // 右編碼器位置 (初始值)
    new Pose2d(),                // 初始位置
    VecBuilder.fill(0.1, 0.1, 0.1), // 狀態估計的標準差 (x, y, theta)
    VecBuilder.fill(0.05),          // 本地量測的標準差 (encoder)
    VecBuilder.fill(0.1)            // 全域量測的標準差 (vision, lidar)
);

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
        this.Left = new DriveModule(DriveMotor.FRONT_LEFT, MotorReverse.FRONT_LEFT,DriveMotor.BACK_LEFT, MotorReverse.BACK_LEFT);
        this.Right = new DriveModule(DriveMotor.FRONT_RIGHT, MotorReverse.FRONT_RIGHT,DriveMotor.BACK_RIGHT, MotorReverse.BACK_RIGHT);
        // headingController.enableContinuousInput(-Math.PI, Math.PI);

    // }
    // public void resetPose(Pose2d pose) {
    //     poseEstimator.resetPosition(gyro.getRotation2d(), getLeftSpeed(), getRightSpeed(), pose);
    // }
    

    // public Pose2d getPose() {
    //     return m_poseEstimator.getEstimatedPosition();
    // }

    //  public void followTrajectory(SwerveSample sample) {
    //     // Get the current pose of the robot
    //     Pose2d pose = getPose();

    //     // Generate the next speeds for the robot
    //     ChassisSpeeds speeds = new ChassisSpeeds(
    //         sample.vx + xController.calculate(pose.getX(), sample.x),
    //         sample.vy + yController.calculate(pose.getY(), sample.y),
    //         sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading)
    //     );

    //     // Apply the generated speeds
    //     driveFieldRelative(speeds);
     }

    public void updateTime() {
        nowTime = Timer.getFPGATimestamp() - timeStates;
        this.timeStates = Timer.getFPGATimestamp();
    }

    public double getLeftSpeed() {
        return this.Left.getVelocity() / 60 / 13.5 * KOP_WHITE_WHEEL * 2 * Math.PI;
    }

    public double getRightSpeed() {
        return this.Right.getVelocity() / 60 / 13.5 * KOP_WHITE_WHEEL * 2 * Math.PI;
    }

    public double leftEncoder() {
        return (this.Left.getVelocity() + this.Left.getVelocity()) / 2;
    }

    public double rightEncoder() {
        return (this.Right.getVelocity()) + this.Right.getVelocity() / 2;
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
        this.updateTime();
    }

}