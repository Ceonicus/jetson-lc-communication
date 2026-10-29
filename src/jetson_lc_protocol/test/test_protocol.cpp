#include <gtest/gtest.h>

#include "jetson_lc_protocol/crc16_modbus.hpp"
#include "jetson_lc_protocol/protocol.hpp"

#include <array>
#include <cstdint>
#include <vector>

using namespace jetson_lc_protocol;

TEST(Crc, StandardModbusVector)
{
  const std::array<std::uint8_t, 9> data{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  EXPECT_EQ(crc16_modbus(data.data(), data.size()), 0x4B37);
}

TEST(Crc, CoverageStartsAfterHeaderAndTransmitsLowFirst)
{
  const auto bytes = encode_poll_request(1, kFunctionStatus);
  const std::array<std::uint8_t, 4> covered{0x03, 0x01, 0x11, 0x00};
  const auto crc = crc16_modbus(covered.data(), covered.size());
  EXPECT_EQ(bytes, (std::vector<std::uint8_t>{0xA5, 0xA5, 0x03, 0x01, 0x11, 0x00,
      static_cast<std::uint8_t>(crc), static_cast<std::uint8_t>(crc >> 8U)}));
  EXPECT_NE(crc, crc16_modbus(bytes.data(), bytes.size() - 2));
}

TEST(Encoding, ExactPollRequests)
{
  EXPECT_EQ(encode_poll_request(1, kFunctionConfiguration),
    (std::vector<std::uint8_t>{0xA5, 0xA5, 0x03, 0x01, 0x10, 0x00, 0x5C, 0x60}));
  EXPECT_EQ(encode_poll_request(1, kFunctionStatus),
    (std::vector<std::uint8_t>{0xA5, 0xA5, 0x03, 0x01, 0x11, 0x00, 0x5D, 0xF0}));
}

TEST(Command, RepresentativeStatesAndReservedBits)
{
  EXPECT_EQ(encode_command_data(CommandState{}),
    (std::vector<std::uint8_t>{0x00, 0x00, 0x00}));
  EXPECT_EQ(encode_command_data(CommandState{0, true, ManualMotion::Down,
      MaintenanceRequest::StartHold}),
    (std::vector<std::uint8_t>{0x00, 0x34, 0x01}));
  EXPECT_EQ(encode_command_request(1, CommandState{0, true, ManualMotion::Down,
      MaintenanceRequest::StartHold}),
    (std::vector<std::uint8_t>{0xA5, 0xA5, 0x05, 0x01, 0x12, 0x00, 0x34, 0x01, 0xEF, 0xF6}));
  EXPECT_EQ(encode_command_data(CommandState{0, true, ManualMotion::Stop,
      MaintenanceRequest::Abort}),
    (std::vector<std::uint8_t>{0x00, 0x24, 0x02}));
  for (const auto motion : {ManualMotion::Stop, ManualMotion::Up, ManualMotion::Down}) {
    const auto bytes = encode_command_data(CommandState{0, true, motion,
        MaintenanceRequest::StartHold});
    EXPECT_EQ(bytes[1] & 0x03U, 0U);
    EXPECT_NE((bytes[1] & 0x18U), 0x18U);
  }
}

TEST(Command, RejectsInvalidCombinationsAndDestination)
{
  std::string reason;
  EXPECT_FALSE(validate_command(CommandState{1, false, ManualMotion::Stop,
      MaintenanceRequest::None}, &reason));
  EXPECT_FALSE(validate_command(CommandState{0, false, ManualMotion::Down,
      MaintenanceRequest::None}, &reason));
  EXPECT_FALSE(validate_command(CommandState{0, false, static_cast<ManualMotion>(9),
      MaintenanceRequest::StartHold}, &reason));
}

TEST(Decoding, ConfigurationIsHighByteFirst)
{
  std::vector<std::uint8_t> data(32, 0);
  data[0] = 0x03; data[1] = 0x84;
  data[2] = 0x06; data[3] = 0x40;
  data[4] = 0x05; data[5] = 0x78;
  data[6] = 0x03; data[7] = 0xE8;
  data[8] = 1; data[9] = 2; data[10] = 0xEF; data[11] = 0xFF;
  const auto result = decode_configuration(Frame{1, 0x10, data});
  ASSERT_TRUE(result);
  EXPECT_EQ(result->rated_capacity_kg, 900);
  EXPECT_EQ(result->car_width_mm, 1600);
  EXPECT_EQ(result->distance_mm, 1400);
  EXPECT_EQ(result->door_width_mm, 1000);
  EXPECT_TRUE(result->has_second_door);
  EXPECT_EQ(result->service_floor_configuration.size(), 16U);
}

TEST(Decoding, StatusBits)
{
  const auto result = decode_status(Frame{1, 0x11, {7, 0x33, 0x06, 0x08}});
  ASSERT_TRUE(result);
  EXPECT_TRUE(result->jetson_operation_permitted);
  EXPECT_TRUE(result->arrived_at_destination);
  EXPECT_TRUE(result->robot_maintenance_mode);
  EXPECT_TRUE(result->lift_abnormal_mode);
  EXPECT_EQ(result->session_state, SessionState::ReadyForJetsonControl);
  EXPECT_EQ(result->abnormal_reason, AbnormalReason::SafetyInterlock);
}

TEST(Values, EveryApprovedSessionAndAbnormalByteIsRepresented)
{
  const std::array<SessionState, 12> states{SessionState::Idle,
    SessionState::PreparationRequestReceived, SessionState::CheckingReadiness,
    SessionState::NotAvailable, SessionState::EnteringRobotMaintenanceMode,
    SessionState::MovingToTopServiceFloor, SessionState::ReadyForJetsonControl,
    SessionState::CompletionRequestReceived, SessionState::ReturningToAutoMode,
    SessionState::ServiceComplete, SessionState::Failed, SessionState::Aborted};
  for (std::size_t i = 0; i < states.size(); ++i) {
    EXPECT_EQ(static_cast<std::uint8_t>(states[i]), i);
  }
  const std::array<AbnormalReason, 13> reasons{AbnormalReason::None,
    AbnormalReason::LiftNotAvailable, AbnormalReason::DoorNotClosed,
    AbnormalReason::PassengerServiceConflict, AbnormalReason::LiftAbnormalMode,
    AbnormalReason::FailedToEnterRobotMaintenanceMode, AbnormalReason::FailedToReturnToAutoMode,
    AbnormalReason::Timeout, AbnormalReason::SafetyInterlock, AbnormalReason::InternalLcError,
    AbnormalReason::ManualTechnicianIntervention, AbnormalReason::AbortRequestedByJetson,
    AbnormalReason::JetsonCommunicationLost};
  for (std::size_t i = 0; i < reasons.size(); ++i) {
    EXPECT_EQ(static_cast<std::uint8_t>(reasons[i]), i);
  }
}

TEST(Parser, PartialMultipleAndNoiseResynchronisation)
{
  FrameParser parser;
  auto first = encode_frame(Frame{1, 0x11, {1, 2, 3, 4}});
  auto second = encode_frame(Frame{1, 0x12, {0}});
  std::vector<std::uint8_t> initial{0x99, 0xA5, 0x33};
  initial.insert(initial.end(), first.begin(), first.begin() + 4);
  EXPECT_TRUE(parser.append(initial).frames.empty());
  std::vector<std::uint8_t> rest(first.begin() + 4, first.end());
  rest.insert(rest.end(), second.begin(), second.end());
  const auto result = parser.append(rest);
  ASSERT_EQ(result.frames.size(), 2U);
  EXPECT_EQ(result.frames[0].function_code, 0x11);
  EXPECT_EQ(result.frames[1].function_code, 0x12);
}

TEST(Parser, InvalidCrcAndLengthResynchronise)
{
  FrameParser parser;
  auto bad = encode_frame(Frame{1, 0x11, {1, 2, 3, 4}});
  bad.back() ^= 0x80;
  auto good = encode_frame(Frame{1, 0x12, {0}});
  bad.insert(bad.end(), {0xA5, 0xA5, 0x01, 0x00});
  bad.insert(bad.end(), good.begin(), good.end());
  const auto result = parser.append(bad);
  EXPECT_EQ(result.crc_errors, 1U);
  EXPECT_GE(result.malformed_frames, 1U);
  ASSERT_EQ(result.frames.size(), 1U);
  EXPECT_TRUE(is_command_ack(result.frames.front()));
}

TEST(Validation, WrongLiftAndUnexpectedFunctionAreRejectedByExpectation)
{
  const Frame wrong_lift{2, 0x11, {1, 0, 0, 0}};
  const Frame wrong_function{1, 0x10, std::vector<std::uint8_t>(32)};
  EXPECT_FALSE(matches_response(wrong_lift, 1, 0x11));
  EXPECT_FALSE(matches_response(wrong_function, 1, 0x11));
  EXPECT_FALSE(decode_status(Frame{1, 0x11, {1, 2, 3}}));
  EXPECT_FALSE(decode_configuration(Frame{1, 0x10, std::vector<std::uint8_t>(31)}));
}
