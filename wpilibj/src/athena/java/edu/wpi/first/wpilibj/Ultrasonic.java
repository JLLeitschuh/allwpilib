/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2016. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj;

import edu.wpi.first.wpilibj.communication.FRCNetworkCommunicationsLibrary.tResourceType;
import edu.wpi.first.wpilibj.communication.UsageReporting;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.tables.ITable;

/**
 * Ultrasonic rangefinder class. The Ultrasonic rangefinder measures absolute distance based on the
 * round-trip time of a ping generated by the controller. These sensors use two transducers, a
 * speaker and a microphone both tuned to the ultrasonic range. A common ultrasonic sensor, the
 * Daventech SRF04 requires a short pulse to be generated on a digital channel. This causes the
 * chirp to be emmitted. A second line becomes high as the ping is transmitted and goes low when the
 * echo is received. The time that the line is high determines the round trip distance (time of
 * flight).
 */
public class Ultrasonic extends SensorBase implements PIDSource, LiveWindowSendable {

  /**
   * The units to return when PIDGet is called.
   */
  public enum Unit {
    /**
     * Use inches for PIDGet.
     */
    kInches,
    /**
     * Use millimeters for PIDGet.
     */
    kMillimeters
  }

  // Time (sec) for the ping trigger pulse.
  private static final double kPingTime = 10 * 1e-6;
  // Priority that the ultrasonic round robin task runs.
  private static final int kPriority = 90;
  // Max time (ms) between readings.
  private static final double kMaxUltrasonicTime = 0.1;
  private static final double kSpeedOfSoundInchesPerSec = 1130.0 * 12.0;
  // head of the ultrasonic sensor list
  private static Ultrasonic m_firstSensor = null;
  // automatic round robin mode
  private static boolean m_automaticEnabled = false;
  private DigitalInput m_echoChannel;
  private DigitalOutput m_pingChannel = null;
  private boolean m_allocatedChannels;
  private boolean m_enabled = false;
  private Counter m_counter = null;
  private Ultrasonic m_nextSensor = null;
  // task doing the round-robin automatic sensing
  private static Thread m_task = null;
  private Unit m_units;
  private static int m_instances = 0;
  protected PIDSourceType m_pidSource = PIDSourceType.kDisplacement;

  /**
   * Background task that goes through the list of ultrasonic sensors and pings each one in turn.
   * The counter is configured to read the timing of the returned echo pulse.
   *
   * <p><b>DANGER WILL ROBINSON, DANGER WILL ROBINSON:</b> This code runs as a task and assumes
   * that none of the ultrasonic sensors will change while it's running. If one does, then this will
   * certainly break. Make sure to disable automatic mode before changing anything with the
   * sensors!!
   */
  private class UltrasonicChecker extends Thread {

    @Override
    public synchronized void run() {
      Ultrasonic ultrasonic = null;
      while (m_automaticEnabled) {
        if (ultrasonic == null) {
          ultrasonic = m_firstSensor;
        }
        if (ultrasonic == null) {
          return;
        }
        if (ultrasonic.isEnabled()) {
          ultrasonic.m_pingChannel.pulse(m_pingChannel.m_channel, (float) kPingTime); // do
          // the
          // ping
        }
        ultrasonic = ultrasonic.m_nextSensor;
        Timer.delay(.1); // wait for ping to return
      }
    }
  }

  /**
   * Initialize the Ultrasonic Sensor. This is the common code that initializes the ultrasonic
   * sensor given that there are two digital I/O channels allocated. If the system was running in
   * automatic mode (round robin) when the new sensor is added, it is stopped, the sensor is added,
   * then automatic mode is restored.
   */
  private synchronized void initialize() {
    if (m_task == null) {
      m_task = new UltrasonicChecker();
    }
    final boolean originalMode = m_automaticEnabled;
    setAutomaticMode(false); // kill task when adding a new sensor
    m_nextSensor = m_firstSensor;
    m_firstSensor = this;

    m_counter = new Counter(m_echoChannel); // set up counter for this
    // sensor
    m_counter.setMaxPeriod(1.0);
    m_counter.setSemiPeriodMode(true);
    m_counter.reset();
    m_enabled = true; // make it available for round robin scheduling
    setAutomaticMode(originalMode);

    m_instances++;
    UsageReporting.report(tResourceType.kResourceType_Ultrasonic, m_instances);
    LiveWindow.addSensor("Ultrasonic", m_echoChannel.getChannel(), this);
  }

  /**
   * Create an instance of the Ultrasonic Sensor. This is designed to supchannel the Daventech SRF04
   * and Vex ultrasonic sensors.
   *
   * @param pingChannel The digital output channel that sends the pulse to initiate the sensor
   *                    sending the ping.
   * @param echoChannel The digital input channel that receives the echo. The length of time that
   *                    the echo is high represents the round trip time of the ping, and the
   *                    distance.
   * @param units       The units returned in either kInches or kMilliMeters
   */
  public Ultrasonic(final int pingChannel, final int echoChannel, Unit units) {
    m_pingChannel = new DigitalOutput(pingChannel);
    m_echoChannel = new DigitalInput(echoChannel);
    m_allocatedChannels = true;
    m_units = units;
    initialize();
  }

  /**
   * Create an instance of the Ultrasonic Sensor. This is designed to supchannel the Daventech SRF04
   * and Vex ultrasonic sensors. Default unit is inches.
   *
   * @param pingChannel The digital output channel that sends the pulse to initiate the sensor
   *                    sending the ping.
   * @param echoChannel The digital input channel that receives the echo. The length of time that
   *                    the echo is high represents the round trip time of the ping, and the
   *                    distance.
   */
  public Ultrasonic(final int pingChannel, final int echoChannel) {
    this(pingChannel, echoChannel, Unit.kInches);
  }

  /**
   * Create an instance of an Ultrasonic Sensor from a DigitalInput for the echo channel and a
   * DigitalOutput for the ping channel.
   *
   * @param pingChannel The digital output object that starts the sensor doing a ping. Requires a
   *                    10uS pulse to start.
   * @param echoChannel The digital input object that times the return pulse to determine the
   *                    range.
   * @param units       The units returned in either kInches or kMilliMeters
   */
  public Ultrasonic(DigitalOutput pingChannel, DigitalInput echoChannel, Unit units) {
    if (pingChannel == null || echoChannel == null) {
      throw new NullPointerException("Null Channel Provided");
    }
    m_allocatedChannels = false;
    m_pingChannel = pingChannel;
    m_echoChannel = echoChannel;
    m_units = units;
    initialize();
  }

  /**
   * Create an instance of an Ultrasonic Sensor from a DigitalInput for the echo channel and a
   * DigitalOutput for the ping channel. Default unit is inches.
   *
   * @param pingChannel The digital output object that starts the sensor doing a ping. Requires a
   *                    10uS pulse to start.
   * @param echoChannel The digital input object that times the return pulse to determine the
   *                    range.
   */
  public Ultrasonic(DigitalOutput pingChannel, DigitalInput echoChannel) {
    this(pingChannel, echoChannel, Unit.kInches);
  }

  /**
   * Destructor for the ultrasonic sensor. Delete the instance of the ultrasonic sensor by freeing
   * the allocated digital channels. If the system was in automatic mode (round robin), then it is
   * stopped, then started again after this sensor is removed (provided this wasn't the last
   * sensor).
   */
  @Override
  public synchronized void free() {
    final boolean wasAutomaticMode = m_automaticEnabled;
    setAutomaticMode(false);
    if (m_allocatedChannels) {
      if (m_pingChannel != null) {
        m_pingChannel.free();
      }
      if (m_echoChannel != null) {
        m_echoChannel.free();
      }
    }

    if (m_counter != null) {
      m_counter.free();
      m_counter = null;
    }

    m_pingChannel = null;
    m_echoChannel = null;

    if (this == m_firstSensor) {
      m_firstSensor = m_nextSensor;
      if (m_firstSensor == null) {
        setAutomaticMode(false);
      }
    } else {
      for (Ultrasonic s = m_firstSensor; s != null; s = s.m_nextSensor) {
        if (this == s.m_nextSensor) {
          s.m_nextSensor = s.m_nextSensor.m_nextSensor;
          break;
        }
      }
    }
    if (m_firstSensor != null && wasAutomaticMode) {
      setAutomaticMode(true);
    }
  }

  /**
   * Turn Automatic mode on/off. When in Automatic mode, all sensors will fire in round robin,
   * waiting a set time between each sensor.
   *
   * @param enabling Set to true if round robin scheduling should start for all the ultrasonic
   *                 sensors. This scheduling method assures that the sensors are non-interfering
   *                 because no two sensors fire at the same time. If another scheduling algorithm
   *                 is preffered, it can be implemented by pinging the sensors manually and waiting
   *                 for the results to come back.
   */
  public void setAutomaticMode(boolean enabling) {
    if (enabling == m_automaticEnabled) {
      return; // ignore the case of no change
    }
    m_automaticEnabled = enabling;

    if (enabling) {
      /* Clear all the counters so no data is valid. No synchronization is
       * needed because the background task is stopped.
       */
      for (Ultrasonic u = m_firstSensor; u != null; u = u.m_nextSensor) {
        u.m_counter.reset();
      }

      // Start round robin task
      m_task.start();
    } else {
      // Wait for background task to stop running
      try {
        m_task.join();
      } catch (InterruptedException ex) {
        Thread.currentThread().interrupt();
        ex.printStackTrace();
      }

      /* Clear all the counters (data now invalid) since automatic mode is
       * disabled. No synchronization is needed because the background task is
       * stopped.
       */
      for (Ultrasonic u = m_firstSensor; u != null; u = u.m_nextSensor) {
        u.m_counter.reset();
      }
    }
  }

  /**
   * Single ping to ultrasonic sensor. Send out a single ping to the ultrasonic sensor. This only
   * works if automatic (round robin) mode is disabled. A single ping is sent out, and the counter
   * should count the semi-period when it comes in. The counter is reset to make the current value
   * invalid.
   */
  public void ping() {
    setAutomaticMode(false); // turn off automatic round robin if pinging
    // single sensor
    m_counter.reset(); // reset the counter to zero (invalid data now)
    m_pingChannel.pulse(m_pingChannel.m_channel, (float) kPingTime); // do
    // the
    // ping
    // to
    // start
    // getting
    // a
    // single
    // range
  }

  /**
   * Check if there is a valid range measurement. The ranges are accumulated in a counter that
   * will increment on each edge of the echo (return) signal. If the count is not at least 2, then
   * the range has not yet been measured, and is invalid.
   *
   * @return true if the range is valid
   */
  public boolean isRangeValid() {
    return m_counter.get() > 1;
  }

  /**
   * Get the range in inches from the ultrasonic sensor. If there is no valid value yet, i.e. at
   * least one measurement hasn't completed, then return 0.
   *
   * @return double Range in inches of the target returned from the ultrasonic sensor.
   */
  public double getRangeInches() {
    if (isRangeValid()) {
      return m_counter.getPeriod() * kSpeedOfSoundInchesPerSec / 2.0;
    } else {
      return 0;
    }
  }

  /**
   * Get the range in millimeters from the ultrasonic sensor. If there is no valid value yet, i.e.
   * at least one measurement hasn't completed, then return 0.
   *
   * @return double Range in millimeters of the target returned by the ultrasonic sensor.
   */
  public double getRangeMM() {
    return getRangeInches() * 25.4;
  }

  @Override
  public void setPIDSourceType(PIDSourceType pidSource) {
    if (!pidSource.equals(PIDSourceType.kDisplacement)) {
      throw new IllegalArgumentException("Only displacement PID is allowed for ultrasonics.");
    }
    m_pidSource = pidSource;
  }

  @Override
  public PIDSourceType getPIDSourceType() {
    return m_pidSource;
  }

  /**
   * Get the range in the current DistanceUnit for the PIDSource base object.
   *
   * @return The range in DistanceUnit
   */
  @Override
  public double pidGet() {
    switch (m_units) {
      case kInches:
        return getRangeInches();
      case kMillimeters:
        return getRangeMM();
      default:
        return 0.0;
    }
  }

  /**
   * Set the current DistanceUnit that should be used for the PIDSource base object.
   *
   * @param units The DistanceUnit that should be used.
   */
  public void setDistanceUnits(Unit units) {
    m_units = units;
  }

  /**
   * Get the current DistanceUnit that is used for the PIDSource base object.
   *
   * @return The type of DistanceUnit that is being used.
   */
  public Unit getDistanceUnits() {
    return m_units;
  }

  /**
   * Is the ultrasonic enabled.
   *
   * @return true if the ultrasonic is enabled
   */
  public boolean isEnabled() {
    return m_enabled;
  }

  /**
   * Set if the ultrasonic is enabled.
   *
   * @param enable set to true to enable the ultrasonic
   */
  public void setEnabled(boolean enable) {
    m_enabled = enable;
  }

  /*
   * Live Window code, only does anything if live window is activated.
   */
  @Override
  public String getSmartDashboardType() {
    return "Ultrasonic";
  }

  private ITable m_table;

  @Override
  public void initTable(ITable subtable) {
    m_table = subtable;
    updateTable();
  }

  @Override
  public ITable getTable() {
    return m_table;
  }

  @Override
  public void updateTable() {
    if (m_table != null) {
      m_table.putNumber("Value", getRangeInches());
    }
  }

  @Override
  public void startLiveWindowMode() {
  }

  @Override
  public void stopLiveWindowMode() {
  }
}
