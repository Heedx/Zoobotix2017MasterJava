package org.usfirst.frc.team6002.robot.subsystems;

import com.ctre.CANTalon;
import org.usfirst.frc.team6002.robot.Constants;
import org.usfirst.frc.team6002.lib.util.Rotation2d;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Turret subsystem controls the direction the ball is fired. On the Turret
 * assembly is the Hood and Flywheel. The Turret can only rotate within 240
 * degrees (+/- 120), and mechanical bumper switches indicate when the
 * mechanical limits are reached. This is part of the Superstructure superclass.
 * 
 * The ball is first picked up with the Intake then is fed to the Flywheel with
 * the HoodRoller. The Turret controls the direction that the ball is fired at.
 * Finally, the Hood controls the output angle and trajectory of the shot.
 * 
 * @see Flywheel
 * @see Hood
 * @see HoodRoller
 * @see Intake
 * @see Superstructure
 */
public class GearArm extends Subsystem {
    private CANTalon gearArmMotor;

    GearArm() {
        // The turret has one Talon.
        gearArmMotor = new CANTalon(Constants.kTurretTalonId);
        gearArmMotor.enableBrakeMode(true);
//        gearArmMotor.enableLimitSwitch(true, true);
//        gearArmMotor.ConfigFwdLimitSwitchNormallyOpen(true);
//        gearArmMotor.ConfigRevLimitSwitchNormallyOpen(true);
        gearArmMotor.setStatusFrameRateMs(CANTalon.StatusFrameRate.Feedback, 10);
        gearArmMotor.setFeedbackDevice(CANTalon.FeedbackDevice.CtreMagEncoder_Relative);
        if (gearArmMotor.isSensorPresent(
                CANTalon.FeedbackDevice.CtreMagEncoder_Relative) != CANTalon.FeedbackDeviceStatus.FeedbackStatusPresent) {
            DriverStation.reportError("Could not detect turret encoder!", false);
        }

        gearArmMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);

        gearArmMotor.setPID(Constants.kGearArmMotorKp, Constants.kGearArmMotorKi, Constants.kGearArmMotorKd, Constants.kGearArmMotorKf,
                Constants.kGearArmMotorIZone, Constants.kGearArmMotorRampRate, 0);
        gearArmMotor.setProfile(0);
        gearArmMotor.reverseSensor(true);
        gearArmMotor.reverseOutput(false);

        // We use soft limits to make sure the turret doesn't try to spin too
        // far.
//        gearArmMotor.enableForwardSoftLimit(true);
//        gearArmMotor.enableReverseSoftLimit(true);
//        gearArmMotor.setForwardSoftLimit(Constants.kSoftMaxTurretAngle / (360.0 * Constants.kTurretRotationsPerTick));
//        gearArmMotor.setReverseSoftLimit(Constants.kSoftMinTurretAngle / (360.0 * Constants.kTurretRotationsPerTick));
    }

    // Set the desired angle of the turret (and put it into position control
    // mode if it isn't already).
    synchronized void setDesiredAngle(Rotation2d angle) {
        gearArmMotor.changeControlMode(CANTalon.TalonControlMode.Position);
        gearArmMotor.set(angle.getRadians() / (2 * Math.PI * Constants.kTurretRotationsPerTick));
    }

    // Manually move the turret (and put it into vbus mode if it isn't already).
    synchronized void setOpenLoop(double speed) {
        gearArmMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
        gearArmMotor.set(speed);
    }

    // Tell the Talon it is at a given position.
    synchronized void reset(Rotation2d actual_rotation) {
        gearArmMotor.setPosition(actual_rotation.getRadians() / (2 * Math.PI * Constants.kTurretRotationsPerTick));
    }

    public synchronized Rotation2d getAngle() {
        return Rotation2d.fromRadians(Constants.kTurretRotationsPerTick * gearArmMotor.getPosition() * 2 * Math.PI);
    }

//    public synchronized boolean getForwardLimitSwitch() {
//        return gearArmMotor.isFwdLimitSwitchClosed();
//    }
//
//    public synchronized boolean getReverseLimitSwitch() {
//        return gearArmMotor.isRevLimitSwitchClosed();
//    }

    public synchronized double getSetpoint() {
        return gearArmMotor.getSetpoint() * Constants.kTurretRotationsPerTick * 360.0;
    }

    private synchronized double getError() {
        return getAngle().getDegrees() - getSetpoint();
    }

    // We are "OnTarget" if we are in position mode and close to the setpoint.
    public synchronized boolean isOnTarget() {
        return (gearArmMotor.getControlMode() == CANTalon.TalonControlMode.Position
                && Math.abs(getError()) < Constants.kTurretOnTargetTolerance);
    }

    /**
     * @return If the turret is within its mechanical limits and in the right
     *         state.
     */
    public synchronized boolean isSafe() {
        return (gearArmMotor.getControlMode() == CANTalon.TalonControlMode.Position && gearArmMotor.getSetpoint() == 0 && Math.abs(
                getAngle().getDegrees() * Constants.kTurretRotationsPerTick * 360.0) < Constants.kTurretSafeTolerance);
    }

    @Override
    public synchronized void stop() {
        setOpenLoop(0);
    }

    @Override
    public void zeroSensors() {
        reset(new Rotation2d());
    }

	
}