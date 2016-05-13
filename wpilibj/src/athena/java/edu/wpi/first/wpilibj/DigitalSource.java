/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2016. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj;

import edu.wpi.first.wpilibj.hal.DIOJNI;
import edu.wpi.first.wpilibj.util.AllocationException;
import edu.wpi.first.wpilibj.util.CheckedAllocationException;

/**
 * DigitalSource Interface. The DigitalSource represents all the possible inputs for a counter or a
 * quadrature encoder. The source may be either a digital input or an analog input. If the caller
 * just provides a channel, then a digital input will be constructed and freed when finished for the
 * source. The source can either be a digital input or analog trigger but not both.
 */
public abstract class DigitalSource extends InterruptableSensorBase {

  protected static Resource channels = new Resource(kDigitalChannels);
  protected long m_port;
  protected int m_channel;

  protected void initDigitalPort(int channel, boolean input) {

    this.m_channel = channel;

    checkDigitalChannel(this.m_channel);

    try {
      channels.allocate(this.m_channel);
    } catch (CheckedAllocationException ex) {
      throw new AllocationException("Digital input " + this.m_channel + " is already allocated");
    }

    long portPointer = DIOJNI.getPort((byte) channel);
    this.m_port = DIOJNI.initializeDigitalPort(portPointer);
    DIOJNI.allocateDIO(this.m_port, input);
  }

  @Override
  public void free() {
    channels.free(this.m_channel);
    DIOJNI.freeDIO(this.m_port);
    DIOJNI.freeDigitalPort(this.m_port);
    this.m_port = 0;
    this.m_channel = 0;
  }

  /**
   * Get the channel routing number.
   *
   * @return channel routing number
   */
  @Override
  public int getChannelForRouting() {
    return this.m_channel;
  }

  /**
   * Get the module routing number.
   *
   * @return 0
   */
  @Override
  public byte getModuleForRouting() {
    return 0;
  }

  /**
   * Is this an analog trigger.
   *
   * @return true if this is an analog trigger
   */
  @Override
  public boolean getAnalogTriggerForRouting() {
    return false;
  }
}
