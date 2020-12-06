package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.Hardware.INTAKE_LEFT_MOTOR_CHANNEL;
import static frc.robot.Constants.Hardware.INTAKE_RIGHT_MOTOR_CHANNEL;

public class Intake extends SubsystemBase {

    private PWMVictorSPX leftIntakeMotor = new PWMVictorSPX(INTAKE_LEFT_MOTOR_CHANNEL);
    private PWMVictorSPX rightIntakeMotor = new PWMVictorSPX(INTAKE_RIGHT_MOTOR_CHANNEL);

    public void intake() {
        leftIntakeMotor.set(0.5);
        rightIntakeMotor.set(0.5);
    }

    public void outtake() {
        leftIntakeMotor.set(-0.5);
        rightIntakeMotor.set(-0.5);
    }

    public void off() {
        leftIntakeMotor.set(0);
        rightIntakeMotor.set(0);
    }

}
