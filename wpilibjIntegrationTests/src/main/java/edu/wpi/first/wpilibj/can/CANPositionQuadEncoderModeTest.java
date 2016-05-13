/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2016. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.can;

import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import java.util.logging.Level;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.Timer;

import static org.junit.Assert.assertEquals;

/**
 * Tests the CAN Motor Encoders in QuadEncoder mode.
 *
 * @author jonathanleitschuh
 */
public class CANPositionQuadEncoderModeTest extends AbstractCANTest {
  private static final Logger logger = Logger.getLogger(CANPositionQuadEncoderModeTest.class
      .getName());

  @Override
  protected Logger getClassLogger() {
    return logger;
  }


  /*
   * (non-Javadoc)
   *$
   * @see edu.wpi.first.wpilibj.can.AbstractCANTest#runMotorForward()
   */
  protected void runMotorForward() {
    double postion = getME().getM_motor().getPosition();
    getME().getM_motor().set(postion + 100);
  }


  /*
   * (non-Javadoc)
   *$
   * @see edu.wpi.first.wpilibj.can.AbstractCANTest#runMotorReverse()
   */
  protected void runMotorReverse() {
    double postion = getME().getM_motor().getPosition();
    getME().getM_motor().set(postion - 100);
  }


  @Before
  public void setUp() throws Exception {
    getME().getM_motor().setPositionMode(CANJaguar.kQuadEncoder, 360, 10.0f, 0.01f, 0.0f);
    getME().getM_motor().enableControl(0);
    /* The motor might still have momentum from the previous test. */
    Timer.delay(kStartupTime);
  }

  @Ignore("The encoder initial position is not validated so is sometimes not set properly")
  @Test
  public void testSetEncoderInitialPositionWithEnable() {
    // given
    final double encoderValue = 4823;
    // when
    getME().getM_motor().enableControl(encoderValue);
    getME().getM_motor().disableControl();
    delayTillInCorrectStateWithMessage(Level.FINE, kEncoderSettlingTime, "Encoder value settling",
        new BooleanCheck() {
          @Override
          public boolean getAsBoolean() {
            getME().getM_motor().set(getME().getM_motor().getPosition());
            return Math.abs(getME().getM_motor().getPosition() - encoderValue) < 40;
          }
        });
    // then
    assertEquals(encoderValue, getME().getM_motor().getPosition(), 40);
  }

  /**
   * Test if we can set a position and reach that position with PID control on the Jaguar.
   */
  @Test
  public void testEncoderPositionPIDForward() {

    double setpoint = getME().getM_motor().getPosition() + 1.0f;

    /* It should get to the setpoint within 10 seconds */
    getME().getM_motor().set(setpoint);
    setCANJaguar(kMotorTimeSettling, setpoint);

    assertEquals("CAN Jaguar should have reached setpoint with PID control", setpoint, getME()
        .getM_motor().getPosition(), kEncoderPositionTolerance);
  }

  /**
   * Test if we can set a position and reach that position with PID control on the Jaguar.
   */
  @Test
  public void testEncoderPositionPIDReverse() {

    double setpoint = getME().getM_motor().getPosition() - 1.0f;

    /* It should get to the setpoint within 10 seconds */
    getME().getM_motor().set(setpoint);
    setCANJaguar(kMotorTimeSettling, setpoint);

    assertEquals("CAN Jaguar should have reached setpoint with PID control", setpoint, getME()
        .getM_motor().getPosition(), kEncoderPositionTolerance);
  }


}
