#include <FakeStream.h>
#include <TurboPumpController.h>
#include <unity.h>

unsigned long arduinoFakeMillis = 0;
unsigned long arduinoYieldIncrementMs = 1;

namespace {
const char START_MOTOR_PUMP_COMMAND[] = "0011002306111111019\r";
const char START_PUMPING_STATION_COMMAND[] = "0011001006111111015\r";
const char STOP_PUMPING_STATION_COMMAND[] = "0011001006000000009\r";
const char ENABLE_ROTATION_SPEED_SET_MODE_COMMAND[] = "0011002607001018\r";
const char SET_SPEED_1200_HZ_COMMAND[] = "0011070706008000030\r";

const char READ_ERROR_CODE_COMMAND[] = "0010030302=?101\r";
const char READ_ACTUAL_SPEED_COMMAND[] = "0010030902=?107\r";
const char READ_DRIVE_POWER_COMMAND[] = "0010031602=?105\r";
const char READ_DRIVE_VOLTAGE_COMMAND[] = "0010031302=?102\r";
const char READ_ELECTRONICS_TEMP_COMMAND[] = "0010032602=?106\r";
const char READ_PUMP_BOTTOM_TEMP_COMMAND[] = "0010033002=?101\r";
const char READ_MOTOR_TEMP_COMMAND[] = "0010034602=?108\r";

TurboPumpConfig makeConfig()
{
  TurboPumpConfig config;
  config.defaultTargetSpeedHz = 1200;
  config.maxSpeedHz = 1500;
  config.readySpeedMarginHz = 50;
  config.readyMaxDrivePowerW = 15;
  config.responseTimeoutMs = 5;
  config.statusQuerySettleMs = 0;
  config.commandAckSettleMs = 0;
  return config;
}

void scriptValue(FakeStream &serial, const char *command, int value)
{
  const std::string response = turboValueResponse(value);
  serial.scriptTextResponse(command, response.c_str());
}

void scriptBasicStatus(FakeStream &serial, int errorCode, int speedHz, int drivePowerW)
{
  scriptValue(serial, READ_ERROR_CODE_COMMAND, errorCode);
  scriptValue(serial, READ_ACTUAL_SPEED_COMMAND, speedHz);
  scriptValue(serial, READ_DRIVE_POWER_COMMAND, drivePowerW);
}
}

void setUp()
{
  resetArduinoFakeTime();
}

void tearDown()
{
}

void test_turbo_start_and_stop_send_expected_commands()
{
  FakeStream serial;
  scriptValue(serial, START_MOTOR_PUMP_COMMAND, 111111);
  scriptValue(serial, START_PUMPING_STATION_COMMAND, 222222);
  scriptValue(serial, STOP_PUMPING_STATION_COMMAND, 0);

  TurboPumpController turbo(serial);
  turbo.configure(makeConfig());

  TEST_ASSERT_TRUE(turbo.start());
  TEST_ASSERT_TRUE(turbo.stop());

  TEST_ASSERT_EQUAL_UINT(3, serial.writeCount());
  TEST_ASSERT_EQUAL_STRING(START_MOTOR_PUMP_COMMAND, serial.writeAt(0).c_str());
  TEST_ASSERT_EQUAL_STRING(START_PUMPING_STATION_COMMAND, serial.writeAt(1).c_str());
  TEST_ASSERT_EQUAL_STRING(STOP_PUMPING_STATION_COMMAND, serial.writeAt(2).c_str());
}

void test_turbo_set_speed_builds_scaled_command_and_enables_set_mode()
{
  FakeStream serial;
  scriptValue(serial, SET_SPEED_1200_HZ_COMMAND, 8000);
  scriptValue(serial, ENABLE_ROTATION_SPEED_SET_MODE_COMMAND, 1);

  TurboPumpController turbo(serial);
  turbo.configure(makeConfig());

  TEST_ASSERT_TRUE(turbo.setTargetSpeedHz(1200));
  TEST_ASSERT_EQUAL_UINT(2, serial.writeCount());
  TEST_ASSERT_EQUAL_STRING(SET_SPEED_1200_HZ_COMMAND, serial.writeAt(0).c_str());
  TEST_ASSERT_EQUAL_STRING(ENABLE_ROTATION_SPEED_SET_MODE_COMMAND, serial.writeAt(1).c_str());
}

void test_turbo_read_full_status_parses_all_values()
{
  FakeStream serial;
  scriptValue(serial, READ_ERROR_CODE_COMMAND, 0);
  scriptValue(serial, READ_ACTUAL_SPEED_COMMAND, 1198);
  scriptValue(serial, READ_DRIVE_POWER_COMMAND, 12);
  scriptValue(serial, READ_DRIVE_VOLTAGE_COMMAND, 24);
  scriptValue(serial, READ_ELECTRONICS_TEMP_COMMAND, 31);
  scriptValue(serial, READ_PUMP_BOTTOM_TEMP_COMMAND, 32);
  scriptValue(serial, READ_MOTOR_TEMP_COMMAND, 33);

  TurboPumpController turbo(serial);
  turbo.configure(makeConfig());

  TurboPumpStatus status;
  TEST_ASSERT_TRUE(turbo.readFullStatus(status));
  TEST_ASSERT_TRUE(status.valid);
  TEST_ASSERT_EQUAL_INT(0, status.errorCode);
  TEST_ASSERT_EQUAL_INT(1198, status.actualSpeedHz);
  TEST_ASSERT_EQUAL_INT(12, status.drivePowerW);
  TEST_ASSERT_EQUAL_INT(24, status.driveVoltageV);
  TEST_ASSERT_EQUAL_INT(31, status.electronicsTempC);
  TEST_ASSERT_EQUAL_INT(32, status.pumpBottomTempC);
  TEST_ASSERT_EQUAL_INT(33, status.motorTempC);
  TEST_ASSERT_EQUAL_STRING("000033", status.lastValue);
}

void test_turbo_is_ready_requires_error_speed_and_power_thresholds()
{
  {
    FakeStream serial;
    scriptBasicStatus(serial, 0, 1200, 10);
    TurboPumpController turbo(serial);
    turbo.configure(makeConfig());
    TurboPumpStatus status;
    TEST_ASSERT_TRUE(turbo.isReady(1200, status));
  }

  {
    FakeStream serial;
    scriptBasicStatus(serial, 0, 1150, 10);
    TurboPumpController turbo(serial);
    turbo.configure(makeConfig());
    TurboPumpStatus status;
    TEST_ASSERT_FALSE(turbo.isReady(1200, status));
  }

  {
    FakeStream serial;
    scriptBasicStatus(serial, 0, 1200, 15);
    TurboPumpController turbo(serial);
    turbo.configure(makeConfig());
    TurboPumpStatus status;
    TEST_ASSERT_FALSE(turbo.isReady(1200, status));
  }

  {
    FakeStream serial;
    scriptBasicStatus(serial, 7, 1200, 10);
    TurboPumpController turbo(serial);
    turbo.configure(makeConfig());
    TurboPumpStatus status;
    TEST_ASSERT_FALSE(turbo.isReady(1200, status));
  }
}

void test_turbo_status_timeout_marks_status_invalid()
{
  FakeStream serial;
  TurboPumpController turbo(serial);
  turbo.configure(makeConfig());

  TurboPumpStatus status;
  TEST_ASSERT_FALSE(turbo.readBasicStatus(status));
  TEST_ASSERT_FALSE(status.valid);
  TEST_ASSERT_EQUAL_UINT(3, serial.writeCount());
  TEST_ASSERT_EQUAL_STRING(READ_ERROR_CODE_COMMAND, serial.writeAt(0).c_str());
  TEST_ASSERT_EQUAL_STRING(READ_ACTUAL_SPEED_COMMAND, serial.writeAt(1).c_str());
  TEST_ASSERT_EQUAL_STRING(READ_DRIVE_POWER_COMMAND, serial.writeAt(2).c_str());
}

void test_turbo_malformed_response_is_rejected()
{
  FakeStream serial;
  serial.scriptTextResponse(READ_ERROR_CODE_COMMAND, "short\r");
  scriptValue(serial, READ_ACTUAL_SPEED_COMMAND, 1200);
  scriptValue(serial, READ_DRIVE_POWER_COMMAND, 10);

  TurboPumpController turbo(serial);
  turbo.configure(makeConfig());

  TurboPumpStatus status;
  TEST_ASSERT_FALSE(turbo.readBasicStatus(status));
  TEST_ASSERT_FALSE(status.valid);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_turbo_start_and_stop_send_expected_commands);
  RUN_TEST(test_turbo_set_speed_builds_scaled_command_and_enables_set_mode);
  RUN_TEST(test_turbo_read_full_status_parses_all_values);
  RUN_TEST(test_turbo_is_ready_requires_error_speed_and_power_thresholds);
  RUN_TEST(test_turbo_status_timeout_marks_status_invalid);
  RUN_TEST(test_turbo_malformed_response_is_rejected);
  return UNITY_END();
}
