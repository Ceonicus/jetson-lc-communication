#include <gtest/gtest.h>

#include "jetson_lc_tools/mock_lc_logic.hpp"

using jetson_lc_protocol::Frame;

TEST(MockLc, RepliesToAllPhaseOneFunctions)
{
  jetson_lc_tools::MockLcLogic mock;
  ASSERT_TRUE(mock.reply(Frame{1, 0x10, {0}}));
  ASSERT_TRUE(mock.reply(Frame{1, 0x11, {0}}));
  const auto ack = mock.reply(Frame{1, 0x12, {0, 0x24, 1}});
  ASSERT_TRUE(ack);
  EXPECT_TRUE(jetson_lc_protocol::is_command_ack(*ack));
}

TEST(MockLc, HoldClearAbortAndDropTransitions)
{
  jetson_lc_tools::MockLcLogic mock;
  for (int i = 0; i < 6; ++i) {ASSERT_TRUE(mock.reply(Frame{1, 0x12, {0, 0x24, 1}}));}
  EXPECT_EQ(mock.hold_count(), 6U);
  auto ready = mock.reply(Frame{1, 0x11, {0}});
  ASSERT_TRUE(ready);
  EXPECT_EQ(ready->data[2], 0x06);
  ASSERT_TRUE(mock.reply(Frame{1, 0x12, {0, 0, 0}}));
  EXPECT_EQ(mock.reply(Frame{1, 0x11, {0}})->data[2], 0x07);
  EXPECT_EQ(mock.reply(Frame{1, 0x11, {0}})->data[2], 0x08);
  EXPECT_EQ(mock.reply(Frame{1, 0x11, {0}})->data[2], 0x09);
  ASSERT_TRUE(mock.reply(Frame{1, 0x12, {0, 0x24, 2}}));
  EXPECT_EQ(mock.reply(Frame{1, 0x11, {0}})->data[2], 0x0B);
  mock.drop_next();
  EXPECT_FALSE(mock.reply(Frame{1, 0x11, {0}}));
}
