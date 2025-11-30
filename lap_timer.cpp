#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "putm_vcl_interfaces/msg/lap_timer.hpp"
#include <cmath>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class LapTimer : public rclcpp::Node
{
private:
    // Define the states for the lap timer
    enum class State
    {
        WAITING_FOR_START,
        RECORDING_REFERENCE_LAP,
        LAPPING
    };

public:
    LapTimer() : Node("lap_timer"), m_state(State::WAITING_FOR_START)
    {
        // ROS2 Subscribers and Publishers
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/vectornav/gnss", 50, std::bind(&LapTimer::gps_callback, this, std::placeholders::_1));

        lap_timer_pub_ = this->create_publisher<putm_vcl_interfaces::msg::LapTimer>("/putm_vcl/lap_timer", 50);

        // ROS2 Wall timer for periodic publishing
        m_wall_timer = this->create_wall_timer(
            20ms, std::bind(&LapTimer::lap_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "LapTimer initialized, waiting for first start line cross.");
    }

private:
    // -- ROS2 Constructs --
    rclcpp::TimerBase::SharedPtr m_wall_timer;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Publisher<putm_vcl_interfaces::msg::LapTimer>::SharedPtr lap_timer_pub_;

    // -- State Machine --
    State m_state;
    bool m_is_approaching_start = false;
    double m_closest_approach_to_start = 0.0;

    // -- Lap Data --
    uint8_t m_lap_count = 0;
    uint16_t m_last_lap_time_ms = 0;
    uint16_t m_best_lap_time_ms = 0;
    double m_delta_time_s = 0.0;
    rclcpp::Time m_current_lap_start_time;

    struct Sector
    {
        double lat, lon, time_s;
    };
    std::vector<Sector> m_reference_lap_sectors;
    std::vector<Sector> m_best_lap_sectors;

    // -- Constants --
    const double EARTH_RADIUS_M = 6371000.0;
    const double START_FINISH_LAT = 52.239040;
    const double START_FINISH_LON = 16.230369;
    const double START_FINISH_GATE_RADIUS_M = 5.0;
    const double SECTOR_RECORDING_DISTANCE_M = 0.5;

    // -- Utility Functions --
    double degreesToRadians(double degrees)
    {
        return degrees * M_PI / 180.0;
    }

    double haversineDistance(double lat1, double lon1, double lat2, double lon2)
    {
        double dLat = degreesToRadians(lat2 - lat1);
        double dLon = degreesToRadians(lon2 - lon1);
        double a = sin(dLat / 2) * sin(dLat / 2) +
                   cos(degreesToRadians(lat1)) * cos(degreesToRadians(lat2)) *
                       sin(dLon / 2) * sin(dLon / 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));
        return EARTH_RADIUS_M * c;
    }

    // -- State Handlers --

    void handle_start_finish_crossing(double current_lat, double current_lon, const rclcpp::Time &now)
    {
        double distance_to_start = haversineDistance(current_lat, current_lon, START_FINISH_LAT, START_FINISH_LON);

        if (distance_to_start < START_FINISH_GATE_RADIUS_M)
        {
            if (!m_is_approaching_start)
            {
                m_is_approaching_start = true;
                m_closest_approach_to_start = distance_to_start;
            }
            else
            {
                m_closest_approach_to_start = std::min(m_closest_approach_to_start, distance_to_start);
            }
        }
        else
        {
            if (m_is_approaching_start) // Just left the start/finish gate
            {
                process_lap_crossing(now);
            }
            m_is_approaching_start = false;
        }
    }

    void process_lap_crossing(const rclcpp::Time &now)
    {
        // This is the moment we crossed the line
        m_lap_count++;
        RCLCPP_INFO(this->get_logger(), "Crossed start/finish line! Lap: %d", m_lap_count);

        if (m_lap_count > 1)
        {
            double completed_lap_time_s = (now - m_current_lap_start_time).seconds();
            m_last_lap_time_ms = static_cast<uint16_t>(completed_lap_time_s * 1000);

            if (m_best_lap_time_ms == 0 || m_last_lap_time_ms < m_best_lap_time_ms)
            {
                m_best_lap_time_ms = m_last_lap_time_ms;
                m_best_lap_sectors = m_reference_lap_sectors; // The lap just finished becomes the new best
                RCLCPP_INFO(this->get_logger(), "New best lap: %.3f s", completed_lap_time_s);
            }
             RCLCPP_INFO(this->get_logger(), "Lap %d time: %.3f s", m_lap_count -1, completed_lap_time_s);
        }

        // Reset for next lap
        m_current_lap_start_time = now;
        m_reference_lap_sectors.clear();

        // Transition state
        if (m_state == State::WAITING_FOR_START)
        {
            m_state = State::RECORDING_REFERENCE_LAP;
            RCLCPP_INFO(this->get_logger(), "Transitioning to RECORDING_REFERENCE_LAP");
        }
        else if(m_state == State::RECORDING_REFERENCE_LAP)
        {
            m_state = State::LAPPING;
            RCLCPP_INFO(this->get_logger(), "Transitioning to LAPPING");
        }
    }

    void state_recording_reference_lap(double current_lat, double current_lon, const rclcpp::Time &now)
    {
        double time_into_lap_s = (now - m_current_lap_start_time).seconds();
        if (m_reference_lap_sectors.empty() || haversineDistance(current_lat, current_lon, m_reference_lap_sectors.back().lat, m_reference_lap_sectors.back().lon) >= SECTOR_RECORDING_DISTANCE_M)
        {
            m_reference_lap_sectors.push_back({current_lat, current_lon, time_into_lap_s});
        }
    }

    void state_lapping(double current_lat, double current_lon, const rclcpp::Time &now)
    {
        if (m_best_lap_sectors.empty())
        {
            m_delta_time_s = 0.0;
            return;
        }

        double min_dist_to_sector = -1.0;
        int closest_sector_idx = -1;

        for (size_t i = 0; i < m_best_lap_sectors.size(); ++i)
        {
            double d = haversineDistance(current_lat, current_lon, m_best_lap_sectors[i].lat, m_best_lap_sectors[i].lon);
            if (closest_sector_idx == -1 || d < min_dist_to_sector)
            {
                min_dist_to_sector = d;
                closest_sector_idx = i;
            }
        }

        if (closest_sector_idx != -1)
        {
            double current_time_into_lap_s = (now - m_current_lap_start_time).seconds();
            double best_lap_time_at_sector_s = m_best_lap_sectors[closest_sector_idx].time_s;
            m_delta_time_s = current_time_into_lap_s - best_lap_time_at_sector_s;
        }
    }

    // -- Main Callbacks --

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        double current_lat = msg->latitude;
        double current_lon = msg->longitude;
        rclcpp::Time now = this->now();

        // Always check for start/finish crossing regardless of state
        handle_start_finish_crossing(current_lat, current_lon, now);

        // State-specific logic
        switch (m_state)
        {
        case State::WAITING_FOR_START:
            // Do nothing until the first cross
            break;
        case State::RECORDING_REFERENCE_LAP:
            state_recording_reference_lap(current_lat, current_lon, now);
            break;
        case State::LAPPING:
            state_lapping(current_lat, current_lon, now);
            break;
        }
    }

    void lap_timer_callback()
    {
        auto message = putm_vcl_interfaces::msg::LapTimer();
        message.best_lap = m_best_lap_time_ms;
        message.lap_counter = m_lap_count;
        message.last_lap = m_last_lap_time_ms;
        message.delta = static_cast<int16_t>(m_delta_time_s * 1000);
        lap_timer_pub_->publish(message);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LapTimer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
