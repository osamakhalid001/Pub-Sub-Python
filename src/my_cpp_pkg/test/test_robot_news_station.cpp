#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/string.hpp>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

// Copy the RobotNewsStationNode class here or include the header
class RobotNewsStationNode : public rclcpp::Node
{
public:
    RobotNewsStationNode() : Node("robot_news_station"), robot_name_("R2D2")
    {
        publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news", 10);
        timer_ = this->create_wall_timer(0.5s, std::bind(&RobotNewsStationNode::publishNews, this));
        RCLCPP_INFO(this->get_logger(), "Robot News Station has been started");
    }

private:
    void publishNews()
    {
        auto msg = example_interfaces::msg::String();
        msg.data = std::string("Hi, this is ") + robot_name_ + std::string(" from the robot news station.");
        publisher_->publish(msg);
    }

    std::string robot_name_;
    rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// Test fixture class
class RobotNewsStationTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        rclcpp::init(0, nullptr);
    }

    void TearDown() override
    {
        rclcpp::shutdown();
    }
};

// Test 1: Node Creation
TEST_F(RobotNewsStationTest, NodeCreation)
{
    auto node = std::make_shared<RobotNewsStationNode>();
    ASSERT_NE(node, nullptr);
    EXPECT_EQ(node->get_name(), std::string("robot_news_station"));
}

// Test 2: Publisher Creation
TEST_F(RobotNewsStationTest, PublisherExists)
{
    auto node = std::make_shared<RobotNewsStationNode>();
    
    // Get the list of publishers
    auto publishers = node->get_node_topics_interface()->get_publisher_names_and_types_by_node(
        node->get_name(), node->get_namespace());
    
    bool found = false;
    for (const auto& pub : publishers)
    {
        if (pub.first == "/robot_news")
        {
            found = true;
            break;
        }
    }
    
    EXPECT_TRUE(found);
}

// Test 3: Message Publishing
TEST_F(RobotNewsStationTest, MessagePublishing)
{
    auto node = std::make_shared<RobotNewsStationNode>();
    
    // Create a subscriber to receive messages
    std::vector<example_interfaces::msg::String::SharedPtr> received_messages;
    auto subscriber = node->create_subscription<example_interfaces::msg::String>(
        "robot_news", 10,
        [&received_messages](example_interfaces::msg::String::SharedPtr msg) {
            received_messages.push_back(msg);
        });
    
    // Spin for a short duration to allow messages to be published
    auto start_time = std::chrono::steady_clock::now();
    while (received_messages.size() < 3 && 
           std::chrono::steady_clock::now() - start_time < 2s)
    {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(100ms);
    }
    
    // Verify that messages were received
    ASSERT_GT(received_messages.size(), 0);
    
    // Verify message content
    for (const auto& msg : received_messages)
    {
        EXPECT_TRUE(msg->data.find("R2D2") != std::string::npos);
        EXPECT_TRUE(msg->data.find("robot news station") != std::string::npos);
    }
}

// Test 4: Message Content Format
TEST_F(RobotNewsStationTest, MessageContentFormat)
{
    auto node = std::make_shared<RobotNewsStationNode>();
    
    example_interfaces::msg::String::SharedPtr last_msg;
    auto subscriber = node->create_subscription<example_interfaces::msg::String>(
        "robot_news", 10,
        [&last_msg](example_interfaces::msg::String::SharedPtr msg) {
            last_msg = msg;
        });
    
    // Spin until we receive at least one message
    auto start_time = std::chrono::steady_clock::now();
    while (!last_msg && std::chrono::steady_clock::now() - start_time < 2s)
    {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(100ms);
    }
    
    ASSERT_NE(last_msg, nullptr);
    
    std::string expected = "Hi, this is R2D2 from the robot news station.";
    EXPECT_EQ(last_msg->data, expected);
}

// Test 5: Publishing Rate
TEST_F(RobotNewsStationTest, PublishingRate)
{
    auto node = std::make_shared<RobotNewsStationNode>();
    
    std::vector<rclcpp::Time> timestamps;
    auto subscriber = node->create_subscription<example_interfaces::msg::String>(
        "robot_news", 10,
        [&timestamps, &node](example_interfaces::msg::String::SharedPtr msg) {
            timestamps.push_back(node->now());
        });
    
    // Collect messages for ~1.5 seconds
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - start_time < 1500ms)
    {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(50ms);
    }
    
    // Should have received approximately 3 messages (published every 0.5s)
    ASSERT_GE(timestamps.size(), 2);
    
    // Check the time difference between consecutive messages
    for (size_t i = 1; i < timestamps.size(); ++i)
    {
        auto diff = (timestamps[i] - timestamps[i-1]).seconds();
        // Allow some tolerance (0.4s to 0.6s)
        EXPECT_GE(diff, 0.4);
        EXPECT_LE(diff, 0.6);
    }
}

// Test 6: Node Shutdown
TEST_F(RobotNewsStationTest, NodeShutdown)
{
    auto node = std::make_shared<RobotNewsStationNode>();
    
    // Verify node is active
    EXPECT_TRUE(rclcpp::ok());
    
    // The node should be able to be destroyed without issues
    node.reset();
    EXPECT_TRUE(rclcpp::ok());
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}