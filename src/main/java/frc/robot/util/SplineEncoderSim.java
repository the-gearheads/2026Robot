package frc.robot.util;

import com.revrobotics.encoder.DetachedEncoder;
import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SplineEncoderSim {
  private SimDouble m_position;
  private SimDouble m_velocity;
  private SimDouble m_angle;
  private SimDouble m_rawAngle;
  private SimBoolean m_isInverted;
  private SimDouble m_zeroOffset;
  private SimDouble m_positionConversionFactor;
  private SimDouble m_velocityConversionFactor;
  @SuppressWarnings("unused")
  private int id;
  private String simDeviceName;

  /**
   * Create a Spark Flex External Encoder simulation object for a Spark Flex. This will allow you to
   * read/write data from the simulated sensor and view it in the Sim GUI.
   *
   * @param motor The CANSparkFlex associated with the sensor
   */
  public SplineEncoderSim(DetachedEncoder encoder) {
    this.id = encoder.getDeviceId();
    simDeviceName = "Detached MAXSpline Encoder [" + encoder.getDeviceId() + "]";
    setupSimDevice();
  }

  // internal setup helper
  private boolean setupSimDevice() {
    SimDeviceSim detachedEncoderSim = new SimDeviceSim(simDeviceName);
    m_position = detachedEncoderSim.getDouble("Position");
    m_velocity = detachedEncoderSim.getDouble("Velocity");
    m_angle = detachedEncoderSim.getDouble("Angle");
    m_rawAngle = detachedEncoderSim.getDouble("Raw Angle");
    m_isInverted = detachedEncoderSim.getBoolean("Is Inverted");
    m_zeroOffset = detachedEncoderSim.getDouble("Zero Offset");
    m_positionConversionFactor = detachedEncoderSim.getDouble("Position Conversion Factor");
    m_velocityConversionFactor = detachedEncoderSim.getDouble("Velocity Conversion Factor");

    return m_position == null;
  }

  // internal setup helper
  private boolean checkAndSetupSimDevice() {
    if (m_position == null) {
      return setupSimDevice();
    }
    return false;
  }

  /**
   * Set the relative position of the sensor, after your conversion factor
   *
   * @param position the position to set
   */
  public void setPosition(double position) {
    if (checkAndSetupSimDevice()) return;
    m_position.set(position);
  }

  /**
   * Get the relative position of the sensor, with the conversion factor applied
   *
   * @return the position of the sensor
   */
  public double getPosition() {
    if (checkAndSetupSimDevice()) return 0;
    return m_position.get();
  }

  /**
   * Set the velocity of the sensor, after the conversion factor
   *
   * @param velocity the velocity to set
   */
  public void setVelocity(double velocity) {
    if (checkAndSetupSimDevice()) return;
    m_velocity.set(velocity);
  }

  public void setAngle(double angle) {
    if (checkAndSetupSimDevice()) return;
    m_angle.set(angle);
  }

  public void setRawAngle(double rawAngle) {
    if (checkAndSetupSimDevice()) return;
    m_rawAngle.set(rawAngle);
  }

  /**
   * Get the velocity of the sensor, with the conversion factor applied
   *
   * @return the velocity of the sensor
   */
  public double getVelocity() {
    if (checkAndSetupSimDevice()) return 0;
    return m_velocity.get();
  }

  /**
   * Set the inversion state of the sensor
   *
   * @param inverted if the sensor is inverted or not
   */
  public void setInverted(boolean inverted) {
    if (checkAndSetupSimDevice()) return;
    m_isInverted.set(inverted);
  }

  /**
   * Get the inversion state of the sensor
   *
   * @return if the sensor is inverted or not
   */
  public boolean getInverted() {
    if (checkAndSetupSimDevice()) return false;
    return m_isInverted.get();
  }

  /**
   * Set the zero offset of the sensor
   *
   * @param zeroOffset the zero offset to apply
   */
  public void setZeroOffset(double zeroOffset) {
    if (checkAndSetupSimDevice()) return;
    m_zeroOffset.set(zeroOffset);
  }

  /**
   * Get the zero offset of the sensor
   *
   * @return the zero offset of the sensor
   */
  public double getZeroOffset() {
    if (checkAndSetupSimDevice()) return 0;
    return m_zeroOffset.get();
  }

  /**
   * Get the position conversion factor of the sensor (1 by default)
   *
   * @return the conversion factor
   */
  public double getPositionConversionFactor() {
    if (checkAndSetupSimDevice()) return 0;
    return m_positionConversionFactor.get();
  }

  /**
   * Get the velocity conversion factor of the sensor (1 by default)
   *
   * @return the conversion factor
   */
  public double getVelocityConversionFactor() {
    if (checkAndSetupSimDevice()) return 0;
    return m_velocityConversionFactor.get();
  }

  /**
   * Move the sensor at the input velocity
   *
   * @param velocity the velocity of the sensor
   * @param dt the time interval of the calculation
   */
  public void iterate(double velocity, double dt) {
    if (checkAndSetupSimDevice()) return;
    double velocityRPM = velocity / getVelocityConversionFactor();
    m_position.set(m_position.get() + ((velocityRPM / 60) * dt) * getPositionConversionFactor());
    m_velocity.set(velocity);
    m_angle.set(m_position.get());
    m_rawAngle.set(m_position.get());
  }
}
