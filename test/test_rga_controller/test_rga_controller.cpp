#include <FakeStream.h>
#include <RGAController.h>
#include <unity.h>

unsigned long arduinoFakeMillis = 0;
unsigned long arduinoYieldIncrementMs = 1;

namespace {
RGAConfig makeConfig(const uint8_t *masses, uint8_t massCount, bool measureTotalPressure = true)
{
  RGAConfig config;
  config.masses = masses;
  config.massCount = massCount;
  config.noiseFloor = 2;
  config.filamentEmissionMa = 1.0f;
  config.scanResponseTimeoutMs = 5;
  config.statusResponseTimeoutMs = 5;
  config.hardwareCommandResponseTimeoutMs = 20;
  config.commandSettleMs = 0;
  config.maxFilamentOffAttempts = 1;
  config.flushBeforeScan = true;
  config.defaultMaxMass = 100;
  config.noiseFloorTimeoutsMs = nullptr;
  config.noiseFloorTimeoutCount = 0;
  config.measureTotalPressure = measureTotalPressure;
  config.parkAfterCycle = false;
  config.parkOnStop = true;
  return config;
}

void runUntilCycleReady(RGAController &rga, uint8_t maxUpdates = 20)
{
  for (uint8_t i = 0; i < maxUpdates && !rga.cycleReady(); i++) {
    rga.update();
  }
}
}

void setUp()
{
  resetArduinoFakeTime();
}

void tearDown()
{
}

void test_rga_scan_reads_masses_then_total_pressure()
{
  const uint8_t masses[] = {2, 18};
  FakeStream serial;
  serial.scriptResponse("MR2\r", littleEndianInt32(12345));
  serial.scriptResponse("MR18\r", littleEndianInt32(-42));
  serial.scriptResponse("TP?\r", littleEndianInt32(777));

  RGAController rga(serial);
  rga.configure(makeConfig(masses, 2));

  TEST_ASSERT_TRUE(rga.startCycle());
  runUntilCycleReady(rga);

  RGACycleData cycle;
  TEST_ASSERT_TRUE(rga.consumeCycle(cycle));
  TEST_ASSERT_TRUE(cycle.complete);
  TEST_ASSERT_FALSE(cycle.hasTimeout);
  TEST_ASSERT_EQUAL_UINT8(2, cycle.readingCount);
  TEST_ASSERT_EQUAL_UINT8(2, cycle.readings[0].mass);
  TEST_ASSERT_TRUE(cycle.readings[0].valid);
  TEST_ASSERT_EQUAL_INT32(12345, cycle.readings[0].current);
  TEST_ASSERT_EQUAL_UINT8(18, cycle.readings[1].mass);
  TEST_ASSERT_TRUE(cycle.readings[1].valid);
  TEST_ASSERT_EQUAL_INT32(-42, cycle.readings[1].current);
  TEST_ASSERT_TRUE(cycle.totalPressure.valid);
  TEST_ASSERT_EQUAL_INT32(777, cycle.totalPressure.current);

  TEST_ASSERT_EQUAL_UINT(3, serial.writeCount());
  TEST_ASSERT_EQUAL_STRING("MR2\r", serial.writeAt(0).c_str());
  TEST_ASSERT_EQUAL_STRING("MR18\r", serial.writeAt(1).c_str());
  TEST_ASSERT_EQUAL_STRING("TP?\r", serial.writeAt(2).c_str());
}

void test_rga_can_optionally_park_after_cycle()
{
  const uint8_t masses[] = {2};
  FakeStream serial;
  serial.scriptResponse("MR2\r", littleEndianInt32(10));

  RGAConfig config = makeConfig(masses, 1, false);
  config.parkAfterCycle = true;

  RGAController rga(serial);
  rga.configure(config);

  TEST_ASSERT_TRUE(rga.startCycle());
  runUntilCycleReady(rga);

  RGACycleData cycle;
  TEST_ASSERT_TRUE(rga.consumeCycle(cycle));
  TEST_ASSERT_EQUAL_UINT(2, serial.writeCount());
  TEST_ASSERT_EQUAL_STRING("MR2\r", serial.writeAt(0).c_str());
  TEST_ASSERT_EQUAL_STRING("MR0\r", serial.writeAt(1).c_str());
}

void test_rga_accepts_partial_response_across_updates()
{
  const uint8_t masses[] = {28};
  FakeStream serial;
  RGAController rga(serial);
  rga.configure(makeConfig(masses, 1, false));

  const std::vector<uint8_t> bytes = littleEndianInt32(0x01020304);

  TEST_ASSERT_TRUE(rga.startCycle());
  rga.update();
  TEST_ASSERT_EQUAL_STRING("MR28\r", serial.writeAt(0).c_str());

  serial.appendInput({bytes[0], bytes[1]});
  rga.update();
  TEST_ASSERT_TRUE(rga.isAcquiring());
  TEST_ASSERT_FALSE(rga.cycleReady());

  serial.appendInput({bytes[2], bytes[3]});
  rga.update();
  TEST_ASSERT_TRUE(rga.cycleReady());

  RGACycleData cycle;
  TEST_ASSERT_TRUE(rga.consumeCycle(cycle));
  TEST_ASSERT_TRUE(cycle.readings[0].valid);
  TEST_ASSERT_EQUAL_INT32(0x01020304, cycle.readings[0].current);
}

void test_rga_mass_timeout_continues_to_next_mass_and_total_pressure()
{
  const uint8_t masses[] = {2, 18};
  FakeStream serial;
  serial.scriptResponse("MR18\r", littleEndianInt32(88));
  serial.scriptResponse("TP?\r", littleEndianInt32(99));

  RGAController rga(serial);
  rga.configure(makeConfig(masses, 2));

  TEST_ASSERT_TRUE(rga.startCycle());
  rga.update();
  TEST_ASSERT_EQUAL_STRING("MR2\r", serial.writeAt(0).c_str());
  delay(5);
  rga.update();

  runUntilCycleReady(rga);

  RGACycleData cycle;
  TEST_ASSERT_TRUE(rga.consumeCycle(cycle));
  TEST_ASSERT_TRUE(cycle.hasTimeout);
  TEST_ASSERT_EQUAL_UINT8(2, cycle.readingCount);
  TEST_ASSERT_FALSE(cycle.readings[0].valid);
  TEST_ASSERT_TRUE(cycle.readings[0].timedOut);
  TEST_ASSERT_TRUE(cycle.readings[1].valid);
  TEST_ASSERT_EQUAL_INT32(88, cycle.readings[1].current);
  TEST_ASSERT_TRUE(cycle.totalPressure.valid);
  TEST_ASSERT_EQUAL_INT32(99, cycle.totalPressure.current);
}

void test_rga_total_pressure_timeout_sets_cycle_timeout()
{
  const uint8_t masses[] = {40};
  FakeStream serial;
  serial.scriptResponse("MR40\r", littleEndianInt32(123));

  RGAController rga(serial);
  rga.configure(makeConfig(masses, 1));

  TEST_ASSERT_TRUE(rga.startCycle());
  rga.update();
  rga.update();
  rga.update();
  TEST_ASSERT_EQUAL_STRING("TP?\r", serial.writeAt(1).c_str());
  delay(5);
  rga.update();

  RGACycleData cycle;
  TEST_ASSERT_TRUE(rga.consumeCycle(cycle));
  TEST_ASSERT_TRUE(cycle.hasTimeout);
  TEST_ASSERT_TRUE(cycle.readings[0].valid);
  TEST_ASSERT_FALSE(cycle.totalPressure.valid);
  TEST_ASSERT_TRUE(cycle.totalPressure.timedOut);
}

void test_rga_config_validation_rejects_bad_values()
{
  const uint8_t masses[] = {2, 18};
  FakeStream serial;
  RGAController rga(serial);

  RGAConfig config = makeConfig(masses, 2);
  rga.configure(config);
  TEST_ASSERT_TRUE(rga.configValid());

  config = makeConfig(nullptr, 0);
  rga.configure(config);
  TEST_ASSERT_FALSE(rga.configValid());

  uint8_t tooMany[RGA_MAX_MASSES + 1] = {0};
  for (uint8_t i = 0; i < sizeof(tooMany); i++) {
    tooMany[i] = 1;
  }
  config = makeConfig(tooMany, sizeof(tooMany));
  rga.configure(config);
  TEST_ASSERT_FALSE(rga.configValid());

  const uint8_t zeroMass[] = {0};
  config = makeConfig(zeroMass, 1);
  rga.configure(config);
  TEST_ASSERT_FALSE(rga.configValid());

  const uint8_t tooHigh[] = {101};
  config = makeConfig(tooHigh, 1);
  rga.configure(config);
  TEST_ASSERT_FALSE(rga.configValid());

  config = makeConfig(masses, 2);
  config.noiseFloor = 8;
  rga.configure(config);
  TEST_ASSERT_FALSE(rga.configValid());

  config = makeConfig(masses, 2);
  config.filamentEmissionMa = 0.01f;
  rga.configure(config);
  TEST_ASSERT_FALSE(rga.configValid());
}

void test_rga_filament_start_and_stop_send_manual_commands()
{
  const uint8_t masses[] = {2};
  FakeStream serial;
  serial.scriptTextResponse("FL1.25\r", "0\n\r");
  serial.scriptTextResponse("NF3\r", "3\n\r");
  serial.scriptResponse("TP1\r", littleEndianInt32(0));
  serial.scriptTextResponse("CA\r", "0\n\r");

  RGAConfig config = makeConfig(masses, 1);
  config.filamentEmissionMa = 1.25f;
  config.noiseFloor = 3;

  RGAController rga(serial);
  rga.configure(config);

  TEST_ASSERT_TRUE(rga.startFilamentBlocking());
  TEST_ASSERT_EQUAL_UINT(5, serial.writeCount());
  TEST_ASSERT_EQUAL_STRING("MR0\r", serial.writeAt(0).c_str());
  TEST_ASSERT_EQUAL_STRING("FL1.25\r", serial.writeAt(1).c_str());
  TEST_ASSERT_EQUAL_STRING("NF3\r", serial.writeAt(2).c_str());
  TEST_ASSERT_EQUAL_STRING("TP1\r", serial.writeAt(3).c_str());
  TEST_ASSERT_EQUAL_STRING("CA\r", serial.writeAt(4).c_str());

  serial.clearWrites();
  serial.scriptResponse("TP0\r", littleEndianInt32(0));
  serial.scriptStatusOk("FL0.00\r");
  serial.scriptTextResponse("FL?\r", "0.00\r");

  TEST_ASSERT_TRUE(rga.stopFilamentBlocking());
  TEST_ASSERT_EQUAL_UINT(4, serial.writeCount());
  TEST_ASSERT_EQUAL_STRING("MR0\r", serial.writeAt(0).c_str());
  TEST_ASSERT_EQUAL_STRING("TP0\r", serial.writeAt(1).c_str());
  TEST_ASSERT_EQUAL_STRING("FL0.00\r", serial.writeAt(2).c_str());
  TEST_ASSERT_EQUAL_STRING("FL?\r", serial.writeAt(3).c_str());
}

void test_rga_filament_start_waits_longer_for_hardware_status_commands()
{
  const uint8_t masses[] = {2};
  FakeStream serial;
  serial.scriptResponseAfter("FL1.25\r", std::vector<uint8_t>{0}, 10);
  serial.scriptTextResponse("NF3\r", "3\n\r");
  serial.scriptResponse("TP1\r", littleEndianInt32(0));
  serial.scriptResponseAfter("CA\r", std::vector<uint8_t>{0}, 10);

  RGAConfig config = makeConfig(masses, 1);
  config.filamentEmissionMa = 1.25f;
  config.noiseFloor = 3;

  RGAController rga(serial);
  rga.configure(config);

  TEST_ASSERT_TRUE(rga.startFilamentBlocking());
  TEST_ASSERT_EQUAL_STRING("MR0\r", serial.writeAt(0).c_str());
  TEST_ASSERT_EQUAL_STRING("FL1.25\r", serial.writeAt(1).c_str());
  TEST_ASSERT_EQUAL_STRING("CA\r", serial.writeAt(4).c_str());
}

void test_rga_filament_start_still_times_out_after_hardware_timeout()
{
  const uint8_t masses[] = {2};
  FakeStream serial;

  RGAConfig config = makeConfig(masses, 1);
  config.filamentEmissionMa = 1.25f;
  config.hardwareCommandResponseTimeoutMs = 20;

  RGAController rga(serial);
  rga.configure(config);

  TEST_ASSERT_FALSE(rga.startFilamentBlocking());
  TEST_ASSERT_EQUAL_STRING("MR0\r", serial.writeAt(0).c_str());
  TEST_ASSERT_EQUAL_STRING("FL1.25\r", serial.writeAt(1).c_str());
  TEST_ASSERT_EQUAL_STRING("FL1.25\r", serial.writeAt(2).c_str());
}

void test_rga_status_error_queries_and_parses_error_bytes()
{
  const uint8_t masses[] = {2};
  FakeStream serial;
  serial.scriptTextResponse("FL1.25\r", "2\n\r");
  serial.scriptTextResponse("ER?\r", "2\n\r");
  serial.scriptTextResponse("EF?\r", "64\n\r");

  RGAConfig config = makeConfig(masses, 1);
  config.filamentEmissionMa = 1.25f;

  RGAController rga(serial);
  rga.configure(config);

  TEST_ASSERT_FALSE(rga.startFilamentBlocking());
  const RGAErrorStatus &status = rga.lastErrorStatus();
  TEST_ASSERT_TRUE(status.valid);
  TEST_ASSERT_EQUAL_UINT8(2, status.statusByte);
  TEST_ASSERT_EQUAL_INT(64, status.filamentError);
  TEST_ASSERT_EQUAL_INT(0, status.rs232Error);
}

void test_rga_stop_can_skip_parking_when_configured()
{
  const uint8_t masses[] = {2};
  FakeStream serial;
  serial.scriptResponse("TP0\r", littleEndianInt32(0));
  serial.scriptStatusOk("FL0.00\r");
  serial.scriptTextResponse("FL?\r", "0.00\r");

  RGAConfig config = makeConfig(masses, 1);
  config.parkOnStop = false;

  RGAController rga(serial);
  rga.configure(config);

  TEST_ASSERT_TRUE(rga.stopFilamentBlocking());
  TEST_ASSERT_EQUAL_UINT(3, serial.writeCount());
  TEST_ASSERT_EQUAL_STRING("TP0\r", serial.writeAt(0).c_str());
  TEST_ASSERT_EQUAL_STRING("FL0.00\r", serial.writeAt(1).c_str());
  TEST_ASSERT_EQUAL_STRING("FL?\r", serial.writeAt(2).c_str());
}

void test_rga_read_total_pressure_blocking_enables_flag_and_decodes_current()
{
  const uint8_t masses[] = {2};
  FakeStream serial;
  serial.scriptResponse("TP1\r", littleEndianInt32(0));
  serial.scriptResponse("TP?\r", littleEndianInt32(-123456));

  RGAController rga(serial);
  rga.configure(makeConfig(masses, 1));

  int32_t current = 0;
  TEST_ASSERT_TRUE(rga.readTotalPressureBlocking(current));
  TEST_ASSERT_EQUAL_INT32(-123456, current);
  TEST_ASSERT_EQUAL_UINT(2, serial.writeCount());
  TEST_ASSERT_EQUAL_STRING("TP1\r", serial.writeAt(0).c_str());
  TEST_ASSERT_EQUAL_STRING("TP?\r", serial.writeAt(1).c_str());
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_rga_scan_reads_masses_then_total_pressure);
  RUN_TEST(test_rga_can_optionally_park_after_cycle);
  RUN_TEST(test_rga_accepts_partial_response_across_updates);
  RUN_TEST(test_rga_mass_timeout_continues_to_next_mass_and_total_pressure);
  RUN_TEST(test_rga_total_pressure_timeout_sets_cycle_timeout);
  RUN_TEST(test_rga_config_validation_rejects_bad_values);
  RUN_TEST(test_rga_filament_start_and_stop_send_manual_commands);
  RUN_TEST(test_rga_filament_start_waits_longer_for_hardware_status_commands);
  RUN_TEST(test_rga_filament_start_still_times_out_after_hardware_timeout);
  RUN_TEST(test_rga_status_error_queries_and_parses_error_bytes);
  RUN_TEST(test_rga_stop_can_skip_parking_when_configured);
  RUN_TEST(test_rga_read_total_pressure_blocking_enables_flag_and_decodes_current);
  return UNITY_END();
}
