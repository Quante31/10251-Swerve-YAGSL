package frc.robot.util;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkBase.ControlType;
import frc.robot.Constants.NeoV1;
import frc.robot.SparkMotor;

/**
 * Utility methods for converting Spark setpoints between control domains.
 */
public final class SparkSetpointUtil {
    private SparkSetpointUtil() {}

    /**
     * Converts a Spark setpoint to an RPM-equivalent value.
     *
     * <p>If control type is {@link ControlType#kVelocity}, this returns the
     * original value. For any other control type, this returns:
     * {@code value * NeoV1.kFreeSpeed.in(RPM)}.
     *
     * @param setpoint Spark motor setpoint (value + control type)
     * @return setpoint expressed as velocity-style RPM value
     */
    public static double toVelocityRpm(SparkMotor.Setpoint setpoint) {
        return toVelocityRpm(setpoint.getValue(), setpoint.getControlType());
    }

    /**
     * Converts a value/control pair to an RPM-equivalent value.
     *
     * <p>Conversion rules:
     * <ul>
     *   <li>{@code kVelocity}, {@code kMAXMotionVelocityControl}: return raw value.</li>
     *   <li>{@code kCurrent}, {@code kDutyCycle}, {@code kPosition},
     *       {@code kVoltage}, {@code kMAXMotionPositionControl}:
     *       return {@code value * NeoV1.kFreeSpeed.in(RPM)}.</li>
     * </ul>
     *
     * @param value setpoint value
     * @param controlType Spark control type
     * @return velocity-style RPM value
     */
    public static double toVelocityRpm(double value, ControlType controlType) {
    final double freeSpeedRpm = NeoV1.kFreeSpeed.in(RPM);

    return switch (controlType) {

        case kVelocity, kMAXMotionVelocityControl -> value;
        case kDutyCycle -> value * freeSpeedRpm;
        case kVoltage -> (value / NeoV1.kNominalVoltage) * freeSpeedRpm;
        // TODO Improve
        case kPosition, kMAXMotionPositionControl, kCurrent -> 0.0;
    };
}
}
