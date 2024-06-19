// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private static final double GEAR_RATIO =
            (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
    private static final double COUNTS_PER_MOTOR_SHAFT_REV = 12.0;
    private static final double COUNTS_PER_REVOLUTION = COUNTS_PER_MOTOR_SHAFT_REV * GEAR_RATIO; // 585.0
    private static final double WHEEL_DIAMETER_INCH = 2.3622; // 60 mm

    // The XRP has the left and right motors set to channel 0 and 1 respectively
    private final XRPMotor leftMotor = new XRPMotor(0);
    private final XRPMotor rightMotor = new XRPMotor(1);

    // Assumes a gamepad plugged into channel 0
    private final XboxController controller = new XboxController(0);

    private double startTime;

    // The XRP has onboard encoders that are hardcoded to use DIO pins 4/5 and 6/7 for the left and right
    private final Encoder leftEncoder = new Encoder(4, 5);
    private final Encoder rightEncoder = new Encoder(6, 7);

    // Set up the differential drive controller
    private final DifferentialDrive diffDrive = new DifferentialDrive(leftMotor, rightMotor);

    // Set up the XRPGyro
    private final XRPGyro gyro = new XRPGyro();

    // Set up the BuiltInAccelerometer
    private final BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        rightMotor.setInverted(true);

        // Use inches as unit for encoder distances
        leftEncoder.setDistancePerPulse((Math.PI * WHEEL_DIAMETER_INCH) / COUNTS_PER_REVOLUTION);
        rightEncoder.setDistancePerPulse((Math.PI * WHEEL_DIAMETER_INCH) / COUNTS_PER_REVOLUTION);
    }

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public double getLeftDistanceInch() {
        return leftEncoder.getDistance();
    }

    public double getRightDistanceInch() {
        return rightEncoder.getDistance();
    }

    public double getAverageDistanceInch() {
        return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
    }

    /**
     * This method is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {}
    
    /**
     * This autonomous (along with the chooser code above) shows how to select between different
     * autonomous modes using the dashboard. The sendable chooser code works with the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all the chooser code and
     * uncomment the getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to the switch structure
     * below with additional strings. If using the SendableChooser, make sure to add them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        startTime = Timer.getFPGATimestamp();
        resetEncoders();
    }
    
    
    /** This method is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
//        driveTime(2.0);
        driveDistance(5.0);
    }

    public void driveDistance(double distanceInch) {
        double currentDistanceInch = getAverageDistanceInch();
        if (currentDistanceInch < distanceInch) {
            diffDrive.arcadeDrive(0.5, 0);
        } else {
            diffDrive.arcadeDrive(0, 0);
        }
    }

    public void driveTime(double durationSec) {
        double currentTime = Timer.getFPGATimestamp();
        if (currentTime - startTime < durationSec) {
            diffDrive.arcadeDrive(0.5, 0);
        } else {
            diffDrive.arcadeDrive(0, 0);
        }
    }
    
    
    /** This method is called once when teleop is enabled. */
    @Override
    public void teleopInit() {}
    
    
    /** This method is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        double speed = -controller.getRawAxis(1) * 0.7;
        double turn = -controller.getRawAxis(4) * 0.5;
        diffDrive.arcadeDrive(speed, turn);
    }
    
    
    /** This method is called once when the robot is disabled. */
    @Override
    public void disabledInit() {}
    
    
    /** This method is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {}
    
    
    /** This method is called once when test mode is enabled. */
    @Override
    public void testInit() {}
    
    
    /** This method is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /**
     * The acceleration in the X-axis.
     *
     * @return The acceleration of the XRP along the X-axis in Gs
     */
    public double getAccelX() {
        return accelerometer.getX();
    }

    /**
     * The acceleration in the Y-axis.
     *
     * @return The acceleration of the XRP along the Y-axis in Gs
     */
    public double getAccelY() {
        return accelerometer.getY();
    }

    /**
     * The acceleration in the Z-axis.
     *
     * @return The acceleration of the XRP along the Z-axis in Gs
     */
    public double getAccelZ() {
        return accelerometer.getZ();
    }

    /**
     * Current angle of the XRP around the X-axis.
     *
     * @return The current angle of the XRP in degrees
     */
    public double getGyroAngleX() {
        return gyro.getAngleX();
    }

    /**
     * Current angle of the XRP around the Y-axis.
     *
     * @return The current angle of the XRP in degrees
     */
    public double getGyroAngleY() {
        return gyro.getAngleY();
    }

    /**
     * Current angle of the XRP around the Z-axis.
     *
     * @return The current angle of the XRP in degrees
     */
    public double getGyroAngleZ() {
        return gyro.getAngleZ();
    }

    /** Reset the gyro. */
    public void resetGyro() {
        gyro.reset();
    }
}
