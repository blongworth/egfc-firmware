#include <LanderCore.h>
#include <unity.h>

void test_parse_query_status()
{
  const SurfaceCommand command = parseSurfaceCommand("?");
  TEST_ASSERT_EQUAL(static_cast<int>(SurfaceCommandType::QueryStatus),
                    static_cast<int>(command.type));
}

void test_parse_start_and_stop_commands()
{
  TEST_ASSERT_EQUAL(static_cast<int>(SurfaceCommandType::StartSystem),
                    static_cast<int>(parseSurfaceCommand("!Z11").type));
  TEST_ASSERT_EQUAL(static_cast<int>(SurfaceCommandType::StartTurboOnly),
                    static_cast<int>(parseSurfaceCommand("!Z10").type));
  TEST_ASSERT_EQUAL(static_cast<int>(SurfaceCommandType::StartRgaIfReady),
                    static_cast<int>(parseSurfaceCommand("!Z12").type));
  TEST_ASSERT_EQUAL(static_cast<int>(SurfaceCommandType::StopSystem),
                    static_cast<int>(parseSurfaceCommand("!Z21").type));
  TEST_ASSERT_EQUAL(static_cast<int>(SurfaceCommandType::StopSystem),
                    static_cast<int>(parseSurfaceCommand("!Z22").type));
  TEST_ASSERT_EQUAL(static_cast<int>(SurfaceCommandType::StopFilament),
                    static_cast<int>(parseSurfaceCommand("!ZFS").type));
}

void test_parse_turbo_speed_command()
{
  const SurfaceCommand command = parseSurfaceCommand("!RS1200");
  TEST_ASSERT_EQUAL(static_cast<int>(SurfaceCommandType::SetTurboSpeed),
                    static_cast<int>(command.type));
  TEST_ASSERT_EQUAL_UINT16(1200, command.targetSpeedHz);
}

void test_rejects_malformed_turbo_speed_command()
{
  TEST_ASSERT_EQUAL(static_cast<int>(SurfaceCommandType::Invalid),
                    static_cast<int>(parseSurfaceCommand("!RS12A0").type));
  TEST_ASSERT_EQUAL(static_cast<int>(SurfaceCommandType::Invalid),
                    static_cast<int>(parseSurfaceCommand("!RS12000").type));
}

void test_rejects_extra_data_on_fixed_commands()
{
  TEST_ASSERT_EQUAL(static_cast<int>(SurfaceCommandType::Invalid),
                    static_cast<int>(parseSurfaceCommand("?1").type));
  TEST_ASSERT_EQUAL(static_cast<int>(SurfaceCommandType::Invalid),
                    static_cast<int>(parseSurfaceCommand("!Z110").type));
}

void test_parse_time_sync_command()
{
  const SurfaceCommand command = parseSurfaceCommand("T1735689601");
  TEST_ASSERT_EQUAL(static_cast<int>(SurfaceCommandType::TimeSync),
                    static_cast<int>(command.type));
  TEST_ASSERT_EQUAL_UINT32(1735689601UL, command.unixTime);
}

void test_rejects_invalid_time_sync_command()
{
  TEST_ASSERT_EQUAL(static_cast<int>(SurfaceCommandType::Invalid),
                    static_cast<int>(parseSurfaceCommand("T1735689600").type));
  TEST_ASSERT_EQUAL(static_cast<int>(SurfaceCommandType::Invalid),
                    static_cast<int>(parseSurfaceCommand("T123").type));
  TEST_ASSERT_EQUAL(static_cast<int>(SurfaceCommandType::Invalid),
                    static_cast<int>(parseSurfaceCommand("T4294967295").type));
  TEST_ASSERT_EQUAL(static_cast<int>(SurfaceCommandType::Invalid),
                    static_cast<int>(parseSurfaceCommand("T1735689601abc").type));
}

void test_state_status_codes()
{
  TEST_ASSERT_EQUAL(0, stateStatusCode(LanderState::Idle));
  TEST_ASSERT_EQUAL(1, stateStatusCode(LanderState::Starting));
  TEST_ASSERT_EQUAL(1, stateStatusCode(LanderState::TurboRunning));
  TEST_ASSERT_EQUAL(4, stateStatusCode(LanderState::RgaReadyCheck));
  TEST_ASSERT_EQUAL(2, stateStatusCode(LanderState::Measuring));
  TEST_ASSERT_EQUAL(3, stateStatusCode(LanderState::Stopping));
  TEST_ASSERT_EQUAL(5, stateStatusCode(LanderState::Error));
}

void test_state_on_off_mapping()
{
  TEST_ASSERT_FALSE(stateOnOff(LanderState::Idle));
  TEST_ASSERT_TRUE(stateOnOff(LanderState::Starting));
  TEST_ASSERT_TRUE(stateOnOff(LanderState::TurboRunning));
  TEST_ASSERT_TRUE(stateOnOff(LanderState::RgaReadyCheck));
  TEST_ASSERT_TRUE(stateOnOff(LanderState::Measuring));
  TEST_ASSERT_FALSE(stateOnOff(LanderState::Stopping));
  TEST_ASSERT_FALSE(stateOnOff(LanderState::Error));
}

void test_format_rga_mass_rows()
{
  char row[64];
  TEST_ASSERT_TRUE(formatRgaMassRow(row, sizeof(row), "2026-05-08T12:00:00Z", 28, -42, true));
  TEST_ASSERT_EQUAL_STRING("R:2026-05-08T12:00:00Z,28,-42", row);

  TEST_ASSERT_TRUE(formatRgaMassRow(row, sizeof(row), "2026-05-08T12:00:00Z", 18, 0, false));
  TEST_ASSERT_EQUAL_STRING("R:2026-05-08T12:00:00Z,18,timeout", row);
}

void test_format_rga_total_pressure_rows()
{
  char row[64];
  TEST_ASSERT_TRUE(formatRgaTotalPressureRow(row, sizeof(row), "2026-05-08T12:00:00Z", 123456, true));
  TEST_ASSERT_EQUAL_STRING("TP:2026-05-08T12:00:00Z,123456", row);

  TEST_ASSERT_TRUE(formatRgaTotalPressureRow(row, sizeof(row), "2026-05-08T12:00:00Z", 0, false));
  TEST_ASSERT_EQUAL_STRING("TP:2026-05-08T12:00:00Z,timeout", row);
}

void test_format_rga_rows_report_truncation()
{
  char row[8];
  TEST_ASSERT_FALSE(formatRgaMassRow(row, sizeof(row), "2026-05-08T12:00:00Z", 28, 1, true));
  TEST_ASSERT_FALSE(formatRgaTotalPressureRow(row, sizeof(row), "2026-05-08T12:00:00Z", 1, true));
}

void test_data_file_rotation_window()
{
  bool createdInWindow = false;

  TEST_ASSERT_FALSE(shouldCreateDataFileForRotation(7, 10, 4, 10, createdInWindow));
  TEST_ASSERT_FALSE(createdInWindow);

  TEST_ASSERT_TRUE(shouldCreateDataFileForRotation(8, 10, 4, 10, createdInWindow));
  TEST_ASSERT_TRUE(createdInWindow);

  TEST_ASSERT_FALSE(shouldCreateDataFileForRotation(8, 10, 4, 10, createdInWindow));
  TEST_ASSERT_TRUE(createdInWindow);

  TEST_ASSERT_FALSE(shouldCreateDataFileForRotation(8, 11, 4, 10, createdInWindow));
  TEST_ASSERT_FALSE(createdInWindow);

  TEST_ASSERT_FALSE(shouldCreateDataFileForRotation(12, 10, 0, 10, createdInWindow));
}

void test_turbo_fault_check_decisions()
{
  uint8_t badCount = 0;

  TEST_ASSERT_EQUAL(static_cast<int>(TurboFaultDecision::None),
                    static_cast<int>(updateTurboFaultCheck(false, false, 2, badCount)));
  TEST_ASSERT_EQUAL_UINT8(1, badCount);

  TEST_ASSERT_EQUAL(static_cast<int>(TurboFaultDecision::None),
                    static_cast<int>(updateTurboFaultCheck(false, false, 2, badCount)));
  TEST_ASSERT_EQUAL_UINT8(2, badCount);

  TEST_ASSERT_EQUAL(static_cast<int>(TurboFaultDecision::Shutdown),
                    static_cast<int>(updateTurboFaultCheck(false, false, 2, badCount)));
  TEST_ASSERT_EQUAL_UINT8(0, badCount);

  badCount = 2;
  TEST_ASSERT_EQUAL(static_cast<int>(TurboFaultDecision::ResetWindow),
                    static_cast<int>(updateTurboFaultCheck(false, true, 2, badCount)));
  TEST_ASSERT_EQUAL_UINT8(1, badCount);

  TEST_ASSERT_EQUAL(static_cast<int>(TurboFaultDecision::ResetWindow),
                    static_cast<int>(updateTurboFaultCheck(true, true, 2, badCount)));
  TEST_ASSERT_EQUAL_UINT8(0, badCount);
}

int main()
{
  UNITY_BEGIN();
  RUN_TEST(test_parse_query_status);
  RUN_TEST(test_parse_start_and_stop_commands);
  RUN_TEST(test_parse_turbo_speed_command);
  RUN_TEST(test_rejects_malformed_turbo_speed_command);
  RUN_TEST(test_rejects_extra_data_on_fixed_commands);
  RUN_TEST(test_parse_time_sync_command);
  RUN_TEST(test_rejects_invalid_time_sync_command);
  RUN_TEST(test_state_status_codes);
  RUN_TEST(test_state_on_off_mapping);
  RUN_TEST(test_format_rga_mass_rows);
  RUN_TEST(test_format_rga_total_pressure_rows);
  RUN_TEST(test_format_rga_rows_report_truncation);
  RUN_TEST(test_data_file_rotation_window);
  RUN_TEST(test_turbo_fault_check_decisions);
  return UNITY_END();
}
