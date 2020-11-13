package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Robot.drivetrain;

public class DriveStraight extends CommandBase {

    private double desiredDistance;
    private double power;

    public DriveStraight(double desiredDistance, double power) {
        addRequirements(drivetrain);

        this.desiredDistance = desiredDistance;
        this.power = power;
    }

    @Override
    public void initialize() {
        drivetrain.reset();

        drivetrain.getDriveStraightHeadingPIDController().reset(new TrapezoidProfile.State(0, 0));
    }

    @Override
    public void execute() {
        drivetrain.getDriveStraightHeadingPIDController().setP(SmartDashboard.getNumber("Drive Straight Heading P", 0.2));
        double turnRate = -drivetrain.getDriveStraightHeadingPIDController().calculate(drivetrain.getHeading().getDegrees(), 0);

        SmartDashboard.putNumber("Error", drivetrain.getDriveStraightHeadingPIDController().getPositionError());

        SmartDashboard.putNumber("Turn rate", turnRate);
        drivetrain.setArcadeSpeeds(power, turnRate);
    }

    @Override
    public boolean isFinished() {
        return Math.abs((drivetrain.leftMetersTravelled() + drivetrain.rightMetersTravelled()) / 2) >= desiredDistance;
    }
}
