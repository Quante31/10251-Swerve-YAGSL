package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.subsystems.FeederSubsystem.Speed;
import frc.robot.Constants.NeoV1;


/**
 * Lightweight wrapper around a {@link SparkMax} brushless motor controller.
 *
 * <p>This class exposes common control and telemetry helpers used by subsystems.
 * It does not apply any default configuration automatically; call
 * {@link #configureMotor(SparkMaxConfig, ResetMode, PersistMode)} before running.
 */
public class SparkMotor {

    public class Setpoint {
        private final double value;
        private final ControlType controlType;

        public Setpoint(double value, ControlType controlType) {
            this.value = value;
            this.controlType = controlType;
        }

        public double getValue() {
            return value;
        }

        public ControlType getControlType() {
            return controlType;
        }
    }
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController closedLoopController;

    private Setpoint setpoint;
    /**
     * Creates a Spark MAX wrapper for the given CAN device ID.
     *
     * @param deviceId CAN ID of the controller on the CAN bus
     * @param type motor type
     */
    public SparkMotor(int deviceId, MotorType type) {
        this.motor = new SparkMax(deviceId, type);
        this.encoder = motor.getEncoder();
        this.closedLoopController = motor.getClosedLoopController();
        this.setpoint = null;
    }
    /**
     * Sends a closed-loop setpoint using the requested control type.
     *
     * @param setpoint target value in units expected by {@code ctrl}
     * @param ctrl Spark closed-loop control mode
     */
    public void set(double setpoint, ControlType ctrl) {
        closedLoopController.setSetpoint(setpoint, ctrl);
        this.setSetpoint(setpoint, ctrl);
    }

    /**
     * Convenience velocity command using the feeder speed enum.
     *
     * @param speed feeder speed preset expressed in RPM
     */
    public void setRPM(Speed speed) {
        closedLoopController.setSetpoint(speed.rpm(), ControlType.kVelocity);
        this.setSetpoint(speed.rpm(), ControlType.kVelocity);

    }
    /**
     * Convenience velocity command using the double rpm value.
     *
     * @param speed in double RPM
     */
    public void setRPM(double rpm) {
        closedLoopController.setSetpoint(rpm, ControlType.kVelocity);
        this.setSetpoint(rpm, ControlType.kVelocity);

    }
    /**
     * Open-loop voltage command from normalized percent output.
     *
     * <p>Expected input range is {@code [-1.0, 1.0]}.
     *
     * @param percentOutput normalized output command
     */
    public void setPercentOutput(double percentOutput) {
        motor.setVoltage(Volts.of(percentOutput * NeoV1.kNominalVoltage));
        this.setSetpoint(percentOutput * NeoV1.kNominalVoltage, ControlType.kVoltage);

    }
    /**
     * Sets the setpoint for the motor.
     *
     * @param setpoint target value in units expected by {@code ctrl}
     * @param ctrl Spark closed-loop control mode
     */
    public void setSetpoint(double setpoint, ControlType ctrl) {
        this.setpoint = new Setpoint(setpoint, ctrl);
    }
    /**
     * Applies a Spark MAX configuration.
     *
     * @param config Spark MAX configuration object
     * @param resetMode parameter reset behavior during configure
     * @param persistMode whether parameters are persisted to flash
     */
    public void configureMotor(final SparkMaxConfig config, ResetMode resetMode, PersistMode persistMode) {
        motor.configure(config, resetMode, persistMode);
    }

    /**
     * @return supplier for output current in amps
     */
    public DoubleSupplier getOutputCurrent(){
        return motor::getOutputCurrent;
    } 

    /**
     * @return supplier for motor temperature in Celsius
     */
    public DoubleSupplier getTemperature(){
        return motor::getMotorTemperature;
    } 

    /**
     * @return supplier for relative encoder velocity in RPM
     */
    public DoubleSupplier getVelocity(){
        return encoder::getVelocity;
    }

    /**
     * @return relative encoder velocity in RPM
     */
    public double getVelocityRPM() {
        return encoder.getVelocity();
    }

    /**
     * @return supplier for relative encoder position in position units (rotations by default)
     */
    public DoubleSupplier getPosition(){
        return encoder::getPosition;
    }

    /**
     * @return supplier for applied output in normalized units ({@code -1.0} to {@code 1.0})
     */
    public DoubleSupplier getAppliedOutput(){
        return motor::getAppliedOutput;
    }

    /**
     * @return supplier for measured bus voltage in volts
     */
    public DoubleSupplier getBusVoltage(){
        return motor::getBusVoltage;
    }

    /**
     * @return output current in amps
     */
    public double getOutputCurrentAmps() {
        return motor.getOutputCurrent();
    }

    /**
     * @return measured bus voltage in volts
     */
    public double getBusVoltageVolts() {
        return motor.getBusVoltage();
    }

    /**
     * @return applied motor voltage in volts
     */
    public double getAppliedVoltageVolts() {
        return motor.getAppliedOutput() * motor.getBusVoltage();
    }

    public Setpoint getSetpoint() {
        return this.setpoint;
    }
    
}
