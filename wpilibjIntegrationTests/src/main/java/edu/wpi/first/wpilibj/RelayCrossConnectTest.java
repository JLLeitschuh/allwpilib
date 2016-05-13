/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008-2016. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.logging.Logger;

import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.InvalidValueException;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.fixtures.RelayCrossConnectFixture;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.test.AbstractComsSetup;
import edu.wpi.first.wpilibj.test.TestBench;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/**
 * Tests the {@link RelayCrossConnectFixture}.
 *
 * @author jonathanleitschuh
 */
public class RelayCrossConnectTest extends AbstractComsSetup {
  private static final Logger logger = Logger.getLogger(RelayCrossConnectTest.class.getName());
  private static final NetworkTable table = NetworkTable.getTable("_RELAY_CROSS_CONNECT_TEST_");
  private RelayCrossConnectFixture m_relayFixture;


  @Before
  public void setUp() throws Exception {
    m_relayFixture = TestBench.getRelayCrossConnectFixture();
    m_relayFixture.setup();
    m_relayFixture.getM_relay().initTable(table);
  }

  @After
  public void tearDown() throws Exception {
    m_relayFixture.reset();
    m_relayFixture.teardown();
  }

  @Test
  public void testBothHigh() {
    m_relayFixture.getM_relay().setDirection(Direction.kBoth);
    m_relayFixture.getM_relay().set(Value.kOn);
    m_relayFixture.getM_relay().updateTable();
    assertTrue("Input one was not high when relay set both high", m_relayFixture.getInputOne()
        .get());
    assertTrue("Input two was not high when relay set both high", m_relayFixture.getM_inputTwo()
        .get());
    assertEquals("On", table.getString("Value"));
  }

  @Test
  public void testFirstHigh() {
    m_relayFixture.getM_relay().setDirection(Direction.kBoth);
    m_relayFixture.getM_relay().set(Value.kForward);
    m_relayFixture.getM_relay().updateTable();
    assertFalse("Input one was not low when relay set Value.kForward", m_relayFixture.getInputOne()
        .get());
    assertTrue("Input two was not high when relay set Value.kForward", m_relayFixture
        .getM_inputTwo()
        .get());
    assertEquals("Forward", table.getString("Value"));
  }

  @Test
  public void testSecondHigh() {
    m_relayFixture.getM_relay().setDirection(Direction.kBoth);
    m_relayFixture.getM_relay().set(Value.kReverse);
    m_relayFixture.getM_relay().updateTable();
    assertTrue("Input one was not high when relay set Value.kReverse", m_relayFixture.getInputOne()
        .get());
    assertFalse("Input two was not low when relay set Value.kReverse", m_relayFixture
        .getM_inputTwo()
        .get());
    assertEquals("Reverse", table.getString("Value"));
  }

  @Test(expected = InvalidValueException.class)
  public void testSetValueForwardWithDirectionReverseThrowingException() {
    m_relayFixture.getM_relay().setDirection(Direction.kForward);
    m_relayFixture.getM_relay().set(Value.kReverse);
  }

  @Test(expected = InvalidValueException.class)
  public void testSetValueReverseWithDirectionForwardThrowingException() {
    m_relayFixture.getM_relay().setDirection(Direction.kReverse);
    m_relayFixture.getM_relay().set(Value.kForward);
  }

  @Test
  public void testInitialSettings() {
    assertEquals(Value.kOff, m_relayFixture.getM_relay().get());
    // Initially both outputs should be off
    assertFalse(m_relayFixture.getInputOne().get());
    assertFalse(m_relayFixture.getM_inputTwo().get());
    assertEquals("Off", table.getString("Value"));
  }

  @Override
  protected Logger getClassLogger() {
    return logger;
  }

}
