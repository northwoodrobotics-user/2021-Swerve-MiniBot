package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import org.usfirst.frc3707.Creedence.Robot;
import frc.robot.Constants;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.SwerveWheel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SwerveDrivetrain extends SubsystemBase {

    //Swerve Modules

    private CANCoder frontRightEncoder = new CANCoder(Constants.DriveSubsystem.kFrontRightEncoderID);
    private CANSparkMax frontRightTwistMotor = new CANSparkMax(Constants.DriveSubsystem.kFrontRightTwistMotorCanID, MotorType.kBrushless);
    private PIDController frontRightTwistController = new PIDController(Constants.DriveSubsystem.kSwerveTwistPID_P, 0.0, 0.0);
    private CANSparkMax frontRightDriveMotor = new CANSparkMax(Constants.DriveSubsystem.kFrontRightDriveMotorCanID, MotorType.kBrushless);
    private SwerveWheel frontRightWheel = new SwerveWheel(frontRightTwistController, frontRightEncoder, frontRightTwistMotor, frontRightDriveMotor, Constants.DriveSubsystem.kFrontRightEncoderOffset, "FrontRight");

    private CANCoder frontLeftEncoder = new CANCoder(Constants.DriveSubsystem.kFrontLeftEncoderID);
    private CANSparkMax frontLeftTwistMotor = new CANSparkMax(Constants.DriveSubsystem.kFrontLeftTwistMotorCanID,MotorType.kBrushless);
    private PIDController frontLeftTwistController = new PIDController(Constants.DriveSubsystem.kSwerveTwistPID_P, 0.0, 0.0);
    private CANSparkMax frontLeftDriveMotor = new CANSparkMax(Constants.DriveSubsystem.kFrontLeftDriveMotorCanID, MotorType.kBrushless);
    private SwerveWheel frontLeftWheel = new SwerveWheel(frontLeftTwistController, frontLeftEncoder, frontLeftTwistMotor, frontLeftDriveMotor,Constants.DriveSubsystem.kFrontLeftEncoderOffset, "FrontLeft");

    private CANCoder rearRightEncoder = new CANCoder(Constants.DriveSubsystem.kRearRightEncoderID);
    private CANSparkMax rearRightTwistMotor = new CANSparkMax(Constants.DriveSubsystem.kRearRightTwistMotorCanID, MotorType.kBrushless);
    private PIDController rearRightTwistController = new PIDController(Constants.DriveSubsystem.kSwerveTwistPID_P, 0.0, 0.0);
    private CANSparkMax rearRightDriveMotor = new CANSparkMax(Constants.DriveSubsystem.kRearRightDriveMotorCanID, MotorType.kBrushless);
    private SwerveWheel rearRightWheel = new SwerveWheel(rearRightTwistController, rearRightEncoder, rearRightTwistMotor, rearRightDriveMotor, Constants.DriveSubsystem.kRearRightEncoderOffset, "RearRight");

    private CANCoder rearLeftEncoder = new CANCoder(Constants.DriveSubsystem.kRearLeftEncoderID);
    private CANSparkMax rearLeftTwistMotor = new CANSparkMax(Constants.DriveSubsystem.kRearLeftTwistMotorCanID, MotorType.kBrushless);
    private PIDController rearLeftTwistController = new PIDController(Constants.DriveSubsystem.kSwerveTwistPID_P, 0.0, 0.0);
    private CANSparkMax rearLeftDriveMotor = new CANSparkMax(Constants.DriveSubsystem.kRearLeftDriveMotorCanID, MotorType.kBrushless);
    private SwerveWheel rearLeftWheel = new SwerveWheel(rearLeftTwistController, rearLeftEncoder, rearLeftTwistMotor, rearLeftDriveMotor, Constants.DriveSubsystem.kRearLeftEncoderOffset, "RearLeft");
    
    //end swerve modules

    //swerve object using all swerve parts
    public SwerveDrive swerve = new SwerveDrive(frontRightWheel, frontLeftWheel, rearLeftWheel, rearRightWheel, null);

    private ShuffleboardTab subsystemShuffleboardTab = Shuffleboard.getTab("Drive Subsystem");

    // Shuffleboard 
    private NetworkTableEntry sbFowardInput = subsystemShuffleboardTab.add("Forward Input", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

    private NetworkTableEntry sbSidewaysInput = subsystemShuffleboardTab.add("Sideways Input", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

    private NetworkTableEntry sbrotation = subsystemShuffleboardTab.add("rotation", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

    private NetworkTableEntry sFieldRelative = subsystemShuffleboardTab.add("fieldOriented", false)
    .withWidget(BuiltInWidgets.kBooleanBox)
    .getEntry();

    private NetworkTableEntry sbFLEncoderRaw = subsystemShuffleboardTab.add("FLEncoder Raw", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

    private NetworkTableEntry sbFREncoderRaw = subsystemShuffleboardTab.add("FREncoder Raw", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

    private NetworkTableEntry sbRLEncoderRaw = subsystemShuffleboardTab.add("RLEncoder Raw", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

    private NetworkTableEntry sbRREncoderRaw = subsystemShuffleboardTab.add("RREncoder Raw", 0)
    .withWidget(BuiltInWidgets.kDial)
    .withProperties(Map.of("min", -1, "max", 1))
    .getEntry();

    public void init() {
        System.out.println("Initializing DriveSubsystem");
        setupEncoders();
    }

    private void setupEncoders() {

        //set twist motors to continuous 360 movement
        frontRightTwistController.enableContinuousInput(0.0, 360.0);
        frontLeftTwistController.enableContinuousInput(0.0, 360.0);
        rearLeftTwistController.enableContinuousInput(0.0, 360.0);
        rearRightTwistController.enableContinuousInput(0.0, 360.0);

        // frontRightTwistController.disableContinuousInput();
        // frontLeftTwistController.disableContinuousInput();
        // rearLeftTwistController.disableContinuousInput();
        // rearRightTwistController.disableContinuousInput();

        frontRightTwistController.setTolerance(Constants.DriveSubsystem.kSwerveTwistPIDTolerance);
        frontLeftTwistController.setTolerance(Constants.DriveSubsystem.kSwerveTwistPIDTolerance);
        rearLeftTwistController.setTolerance(Constants.DriveSubsystem.kSwerveTwistPIDTolerance);
        rearRightTwistController.setTolerance(Constants.DriveSubsystem.kSwerveTwistPIDTolerance);

    }

    public void enable() {
        System.out.println("Enabling DriveSubsystem");
        frontRightWheel.enableRotation();
        frontLeftWheel.enableRotation();
        rearRightWheel.enableRotation();
        rearLeftWheel.enableRotation();
    }

    /*public void disable() {
        System.out.println("Disabling DriveSubsystem");
        frontRightWheel.disableRotation();
        frontLeftWheel.disableRotation();
        rearRightWheel.disableRotation();
        rearLeftWheel.disableRotation();
    }*/

    /**
     * Drives the robot based on parameter values
     * 
     * @param directionX Proportional speed at which to move left to right
     * @param directionY Proportional speed at which to move front to back
     * @param rotation   Proportional speed at which to rotate
     * @param useGyro    Boolean for field-oriented driving
     * @param slowSpeed  Boolean for slow mode to make the robot drive slower.
     * @param noPush     Boolean to lock wheels at 45 degree angles, to prevent the
     *                   robot from being pushed in any direction
     */
    public void drive(double directionX, double directionY, double rotation, boolean useGyro, boolean slowSpeed,
            boolean noPush) {
        swerve.drive(directionX * 0.5, directionY * 0.5, rotation * 0.34, false, slowSpeed, noPush);
    }

    //rotates the swerves to a circle and runs the motors at a desired speed
    public void CircleDrive(double speed)
    {
        swerve.CircleRotate(speed);
    }

    public void driveSimple(double speed, double angle)
    {
        swerve.driveSimple(speed, angle);
    }

    public void xMode()
    {
        swerve.XModeActivate();
    }

    /**
     * The function which executes periodically to run the DriveTrain subsystem
     */
    @Override
    public void periodic() {
        publishDataToSmartDashboard();
    }

    private void publishDataToSmartDashboard() {

      // sbFLEncoderRaw.setDouble(m_frontLeft.getRawAngle());
      // sbFREncoderRaw.setDouble(m_frontRight.getRawAngle());
      // sbRLEncoderRaw.setDouble(m_rearLeft.getRawAngle());
      // sbRREncoderRaw.setDouble(m_rearRight.getRawAngle());

        // Publish encoder values to smart dashboard for offset tuning
        SmartDashboard.putNumber("Front Right Encoder", frontRightEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Front Left Encoder", frontLeftEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Back Right Encoder", rearRightEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Back Left Encoder", rearLeftEncoder.getAbsolutePosition());

    }

    
    public void disableFrontRightWheelRotation(){
        frontRightWheel.disableRotation();
    }
    public void disableFrontLeftWheelRotation(){
        frontLeftWheel.disableRotation();
    }
    public void disableRearRightWheelRotation(){
        rearRightWheel.disableRotation();
    }
    public void disableRearLeftWheelRotation(){
        rearLeftWheel.disableRotation();
    }
    public void enableFrontRightWheelRotation(){
        frontRightWheel.enableRotation();
    }
    public void enableFrontLeftWheelRotation(){
        frontLeftWheel.enableRotation();
    }
    public void enableRearRightWheelRotation(){
        rearRightWheel.enableRotation();
    }
    public void enableRearLeftWheelRotation(){
        rearLeftWheel.enableRotation();
    }
}