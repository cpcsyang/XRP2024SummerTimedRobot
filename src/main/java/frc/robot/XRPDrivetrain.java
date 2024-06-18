// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.xrp.XRPMotor;



public class XRPDrivetrain
{
    private static final double GEAR_RATIO =
            (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
    private static final double COUNTS_PER_MOTOR_SHAFT_REV = 12.0;
    private static final double COUNTS_PER_REVOLUTION = COUNTS_PER_MOTOR_SHAFT_REV * GEAR_RATIO; // 585.0
    private static final double WHEEL_DIAMETER_INCH = 2.3622; // 60 mm
    
    // The XRP has the left and right motors set to
    // channel 0 and 1 respectively
    private final XRPMotor leftMotor = new XRPMotor(0);
    private final XRPMotor rightMotor = new XRPMotor(1);
    
    // The XRP has onboard encoders that are hardcoded
    // to use DIO pins 4/5 and 6/7 for the left and right
    private final Encoder leftEncoder = new Encoder(4, 5);
    private final Encoder rightEncoder = new Encoder(6, 7);
    
    // Set up the differential drive controller
    private final DifferentialDrive diffDrive = new DifferentialDrive(leftMotor, rightMotor);
    
    
    /** Creates a new XRPDrivetrain. */
    public XRPDrivetrain()
    {
        // Use inches as unit for encoder distances
        leftEncoder.setDistancePerPulse((Math.PI * WHEEL_DIAMETER_INCH) / COUNTS_PER_REVOLUTION);
        rightEncoder.setDistancePerPulse((Math.PI * WHEEL_DIAMETER_INCH) / COUNTS_PER_REVOLUTION);
        resetEncoders();
        
        // Invert right side since motor is flipped
        rightMotor.setInverted(true);
    }
    
    
    public void arcadeDrive(double xAxisSpeed, double zAxisRotate)
    {
        diffDrive.arcadeDrive(xAxisSpeed, zAxisRotate);
    }
    
    
    public void resetEncoders()
    {
        leftEncoder.reset();
        rightEncoder.reset();
    }
    
    
    public double getLeftDistanceInch()
    {
        return leftEncoder.getDistance();
    }
    
    
    public double getRightDistanceInch()
    {
        return rightEncoder.getDistance();
    }
}
