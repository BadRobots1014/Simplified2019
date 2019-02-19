package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.SpeedController;

public class CANTalonSRX extends TalonSRX implements SpeedController {
    private boolean disabled;

    public CANTalonSRX(int deviceNumber) {
        super(deviceNumber);
    }

    @Override
    public void pidWrite(double output) {

    }

    @Override
    public void set(double speed) {
        if (!disabled)
            super.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public double get() {
        return getMotorOutputPercent();
    }

    @Override
    public void disable() {
        disabled = true;
    }

    @Override
    public void stopMotor() {
        set(0);
    }
}