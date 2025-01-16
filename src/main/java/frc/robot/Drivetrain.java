package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain {
  // Constants
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  private static final double kTrackWidth = 0.381 * 2;
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = -4096;

  // Hardware components
  private final WPI_TalonSRX m_leftLeader = new WPI_TalonSRX();
  private final WPI_TalonSRX m_leftFollower = new WPI_TalonSRX();
  private final WPI_TalonSRX m_rightLeader = new WPI_TalonSRX();
  private final WPI_TalonSRX m_rightFollower = new WPI_TalonSRX();

  @SuppressWarnings("removal")
  private final MotorControllerGroup m_leftGroup = new MotorControllerGroup((MotorController) m_leftLeader, (MotorController) m_leftFollower);
  @SuppressWarnings("removal")
  private final MotorControllerGroup m_rightGroup = new MotorControllerGroup((MotorController) m_rightLeader, (MotorController) m_rightFollower);

  private final Encoder m_leftEncoder = new Encoder(0, 1);
  private final Encoder m_rightEncoder = new Encoder(2, 3);

  private final PIDController m_leftPIDController = new PIDController(8.5, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(8.5, 0, 0);

  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);
  private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);

  // Field2d object for visualization
  private final Field2d m_fieldSim = new Field2d();

  @SuppressWarnings("removal")
  public Drivetrain() {
    // Invert the right side motors
    m_rightGroup.setInverted(true);

    // Set the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
    m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    // Reset encoders
    m_leftEncoder.reset();
    m_rightEncoder.reset();

    // Display field on SmartDashboard
    SmartDashboard.putData("Field", m_fieldSim);
  }

  @SuppressWarnings("removal")
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    var leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    var rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);
    double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
    double rightOutput = m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);

    m_leftGroup.setVoltage(leftOutput + leftFeedforward);
    m_rightGroup.setVoltage(rightOutput + rightFeedforward);
  }

  public void drive(double xSpeed, double rot) {
    setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot)));
  }

  public void updateOdometry() {
    m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
  }

  public void resetOdometry(Pose2d pose) {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    m_odometry.resetPosition(m_gyro.getRotation2d(), new DifferentialDriveWheelPositions(m_leftEncoder.getDistance(), m_rightEncoder.getDistance()), pose);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void periodic() {
    updateOdometry();
  }
}