package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.teleop.DriveCommand;

import static frc.robot.Constants.Hardware.*;
import static frc.robot.Robot.drivetrain;

public class Drivetrain extends SubsystemBase {

    private final Talon leftWheels = new Talon(DRIVE_LEFT_CHANNEL);
    private final Talon rightWheels = new Talon(DRIVE_RIGHT_CHANNEL);

    private final Encoder leftEncoder = new Encoder(new DigitalInput(LEFT_ENCODER_CHANNEL1), new DigitalInput(LEFT_ENCODER_CHANNEL2));
    private final Encoder rightEncoder = new Encoder(new DigitalInput(RIGHT_ENCODER_CHANNEL1), new DigitalInput(RIGHT_ENCODER_CHANNEL2));

    private final AHRS ahrs = new AHRS(SPI.Port.kMXP);

    private final Compressor compressor = new Compressor();
    private final Solenoid pneumaticsShifter = new Solenoid(SHIFTER_CHANNEL);

    private ProfiledPIDController turnPIDController = new ProfiledPIDController(0.05, 0, 0,
            new TrapezoidProfile.Constraints(360, 80));

    private ProfiledPIDController driveStraightHeadingPIDController = new ProfiledPIDController(0.2, 0, 0,
            new TrapezoidProfile.Constraints(360, 80));

    public Drivetrain() {
        leftWheels.setInverted(true);
        leftEncoder.setReverseDirection(true);

        setDistancePerPulse();
        reset();

        compressor.stop();
    }

    public AHRS getAhrs() {
        return ahrs;
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-ahrs.getAngle());  // counter clock wise positive
    }

    public double getDistancePerPulse() {
        return 0.00038963112;
    }

    public ProfiledPIDController getTurnPIDController() {
        return turnPIDController;
    }

    public ProfiledPIDController getDriveStraightHeadingPIDController() {
        return driveStraightHeadingPIDController;
    }

    public double leftMetersTravelled() {
        return leftEncoder.getDistance();
    }

    public double rightMetersTravelled() {
        return rightEncoder.getDistance();
    }

    public void setDistancePerPulse() {
        leftEncoder.setDistancePerPulse(getDistancePerPulse());
        rightEncoder.setDistancePerPulse(getDistancePerPulse());
    }

    public void reset() {
        resetEncoders();
        resetHeading();
    }

    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public void resetHeading() {
        ahrs.zeroYaw();
    }

    public void shiftDown() {
        pneumaticsShifter.set(true);
    }

    public void shiftUp() {
        pneumaticsShifter.set(false);
    }

    public boolean isShiftDown() {
        return pneumaticsShifter.get();
    }

    public void setDutyCycles(double leftDutyCycle, double rightDutyCycle) {
        leftWheels.set(leftDutyCycle);
        rightWheels.set(rightDutyCycle);
    }

    public void setArcadeSpeeds(double xSpeed, double zRotation) {
        xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
        zRotation = Math.copySign(zRotation * zRotation, zRotation);

        double leftMotorOutput;
        double rightMotorOutput;

        xSpeed = Math
                .max(-1.0 + Math.abs(zRotation),
                        Math.min(1.0 - Math.abs(zRotation), xSpeed));

        leftMotorOutput = xSpeed + zRotation;
        rightMotorOutput = xSpeed - zRotation;

        setDutyCycles(leftMotorOutput, rightMotorOutput);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Is shift down", isShiftDown());
        SmartDashboard.putNumber("Left meters", drivetrain.leftMetersTravelled());
        SmartDashboard.putNumber("Right meters", drivetrain.rightMetersTravelled());
        SmartDashboard.putNumber("Angle", drivetrain.getHeading().getDegrees());
    }
}
