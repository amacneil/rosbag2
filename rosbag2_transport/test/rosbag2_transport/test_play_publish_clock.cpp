// Copyright 2021 Amazon.com, Inc. or its affiliates. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gmock/gmock.h>

#include <memory>
#include <utility>
#include <vector>

#include "rosbag2_transport/rosbag2_transport.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "rosbag2_play_test_fixture.hpp"

using namespace ::testing;  // NOLINT

namespace
{
template<typename Duration>
rcutils_duration_value_t as_nanos(Duration d)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(d).count();
}

rcutils_duration_value_t period_for_frequency(double frequency)
{
  return static_cast<rcutils_duration_value_t>(RCUTILS_S_TO_NS(1) / frequency);
}
}  // namespace

TEST_F(RosBag2PlayTestFixture, clock_is_published_at_chosen_frequency)
{
  // Test values
  play_options_.clock_publish_frequency = 20;
  const size_t messages_to_play = 10;
  const auto milliseconds_between_messages = 200;
  // Wait for just a few clock messages, we're checking the time between them, not the total count
  const size_t expected_clock_messages = 4;

  // Fake bag setup
  auto topic_types = std::vector<rosbag2_storage::TopicMetadata>{
    {"topic1", "test_msgs/BasicTypes", "", ""},
  };

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages;
  for (size_t i = 0; i < messages_to_play; i++) {
    auto message = get_messages_basic_types()[0];
    message->int32_value = static_cast<int32_t>(i);
    messages.push_back(
      serialize_test_message("topic1", milliseconds_between_messages * i, message));
  }

  // Player setup
  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, topic_types);
  reader_ = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  sub_->add_subscription<rosgraph_msgs::msg::Clock>(
    "/clock", expected_clock_messages, rclcpp::ClockQoS());
  auto await_received_messages = sub_->spin_subscriptions();

  // Play
  rosbag2_transport::Rosbag2Transport rosbag2_transport(reader_, writer_);
  rosbag2_transport.play(storage_options_, play_options_);
  await_received_messages.get();

  // Check that we got enough messages
  auto received_clock = sub_->get_received_messages<rosgraph_msgs::msg::Clock>("/clock");
  EXPECT_THAT(received_clock, SizeIs(Ge(expected_clock_messages)));

  // Check time deltas between messages
  auto message_times = sub_->get_received_messages_time("/clock");
  const auto expect_clock_delta = period_for_frequency(play_options_.clock_publish_frequency);
  const auto allowed_error = static_cast<rcutils_duration_value_t>(expect_clock_delta * 0.1);
  for (size_t i = 0; i < expected_clock_messages - 1; i++) {
    auto delta = message_times[i + 1] - message_times[i];
    auto error = std::abs(as_nanos(delta) - expect_clock_delta);
    EXPECT_LE(error, allowed_error);
  }
}
