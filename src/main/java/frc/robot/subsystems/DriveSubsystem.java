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
    private final DriveModule frontLeft;
    private final DriveModule frontRight;
    private final DriveModule backLeft;
    private final DriveModule backRight;

    // private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    // private final PoseEstimator poseEstimator = new PoseEstimator<>(null, null, null, null);

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
        this.frontLeft = new DriveModule(DriveMotor.FRONT_LEFT, MotorReverse.FRONT_LEFT);
        this.frontRight = new DriveModule(DriveMotor.FRONT_RIGHT, MotorReverse.FRONT_RIGHT);
        this.backLeft = new DriveModule(DriveMotor.BACK_LEFT, MotorReverse.BACK_LEFT);
        this.backRight = new DriveModule(DriveMotor.BACK_RIGHT, MotorReverse.BACK_RIGHT);

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
        return this.backLeft.getVelocity() / 60 / 13.5 * KOP_WHITE_WHEEL * 2 * Math.PI;
    }

    public double getRightSpeed() {
        return this.backRight.getVelocity() / 60 / 13.5 * KOP_WHITE_WHEEL * 2 * Math.PI;
    }

    public double leftEncoder() {
        return (this.backLeft.getVelocity() + this.frontLeft.getVelocity()) / 2;
    }

    public double rightEncoder() {
        return (this.backRight.getVelocity()) + this.frontRight.getVelocity() / 2;
    }

    public void stopModules() {
        this.frontLeft.stop();
        this.frontRight.stop();
        this.backLeft.stop();
        this.backRight.stop();
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
        this.backLeft.setVoltage(leftVoltage);
        this.frontLeft.setVoltage(leftVoltage);
        this.backRight.setVoltage(rightVoltage);
        this.frontRight.setVoltage(rightVoltage);

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