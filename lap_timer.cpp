#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include <cmath>
#include <chrono>
#include <memory>
#include <sstream>
#include <vector>
#include <fstream>

using namespace std::chrono_literals;

class LapTimer : public rclcpp::Node
{
private:
    enum class State
    {
        WAITING_FOR_START,       // Driving to the start line (Outlap)
        RECORDING_REFERENCE_LAP, // First timed lap (Creating the reference map)
        LAPPING                  // Subsequent laps (Calculating delta vs best lap)
    };
    // Structure to represent a sector
    struct Sector
    {
        double lat, lon, time; // Latitude, longitude, and time
    };
    // Start/finish line coordinates
    const double EARTH_RADIUS = 6371000.0; // Earth's radius in meters
    const double START_LAT = 52.239048;    // Start latitude
    const double START_LON = 16.230333;    // Start longitude
    const double DELTA_DISTANCE = 0.5;     // Minimum distance between sectors
    const double GATE_RADIUS = 10.0;       // Radius to detect crossing the line
public:
    LapTimer() : Node("lap_timer"), state(State::WAITING_FOR_START){
        // Create a subscription to the "/vectornav/gnss" topic with a QoS of 50
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/vectornav/gnss", 50, std::bind(&LapTimer::gps_callback, this, std::placeholders::_1));
            
        // Create publishers for the lap timer delta and time
        // Uncomment line below when using on a car with putm_vcl_interfaces
        // lap_timer_pub = this->create_publisher<putm_vcl_interfaces::msg::LapTimer>("/putm_vcl/lap_timer", 50);
        
        // Create a timer that triggers every 20 milliseconds
        timer_ = this->create_wall_timer(
            20ms, std::bind(&LapTimer::lap_timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "LapTimer Initialized. State: WAITING_FOR_START");
    
    }
private:
    //State machine logic variables
    State state;
    bool is_approaching_start = false;
    double closest_approach = 100.0;
    bool has_crossed_this_pass = false;

    //Lap data
    int lap_count = 0;
    double current_lap_time_start = 0.0;
    double lap_time = 0.0;
    double best_lap_time = 0.0;
    double delta_time_s = 0.0;

    rclcpp::Time current_lap_start_time;

    //Sectors of the current lap lon, lat, time
    std::vector<Sector> current_lap_sectors; 
    std::vector<Sector> best_lap_sectors;


    // Function to convert degrees to radians
    double degreesToRadians(double degrees)
    {
        // Convert degrees to radians using the formula: radians = degrees * pi / 180
        return degrees * M_PI / 180.0;
    }

    // Function to calculate the distance between two GPS points using the Haversine formula
    double haversineDistance(double latitude1, double longitude1, double latitude2, double longitude2)
    {
        // Convert latitudes and longitudes to radians
        double lat1 = degreesToRadians(latitude1);
        double lon1 = degreesToRadians(longitude1);
        double lat2 = degreesToRadians(latitude2);
        double lon2 = degreesToRadians(longitude2);

        // Calculate the differences between latitudes and longitudes
        double dLat = lat2 - lat1;
        double dLon = lon2 - lon1;

        // Calculate the Haversine distance
        double a = sin(dLat / 2) * sin(dLat / 2) +
                   cos(lat1) * cos(lat2) *
                       sin(dLon / 2) * sin(dLon / 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));

        // Return the distance in meters
        return EARTH_RADIUS * c;
    }

    // Checks if the car has crossed the Start/Finish line
    void handle_start_finish_crossing(double current_lat, double current_lon, const rclcpp::Time &now)
    {
        // Calculate the distance between the current position and the start/finish line
        double distance_to_start = haversineDistance(current_lat, current_lon, START_LAT, START_LON);
        // Check if the vehicle is close to the start/finish line
        if (distance_to_start < GATE_RADIUS)
        {
            //Check if we are approaching the line if not set the aproaching flag and calculate the closest approach
            if (!is_approaching_start)
            {
                closest_approach = distance_to_start;
                is_approaching_start = true;
            }
            //
            else
            {
                if (distance_to_start < closest_approach)
                {
                    closest_approach = distance_to_start;
                }
                //If we are not crossing the finish line and the distance_to_start is greater than closest_approach
                else if (!has_crossed_this_pass)
                {

                    RCLCPP_INFO(this->get_logger(), "Start/Finish line crossed!");
                    has_crossed_this_pass = true;
                    process_lap_crossing(now);
                }
            }
        }
        else
        {
            //We are outside the gate radius reset the flags but only when is_approaching_start is still true from previous if 
            //and then we set it to false here so this if only triggers once when leaving the gate area
            if (is_approaching_start)
            {
                RCLCPP_INFO(this->get_logger(), "Left Start/Finish line area.");
                is_approaching_start = false;
                has_crossed_this_pass = false;
                closest_approach = 100.0;
            }
        }
    }

    void process_lap_crossing(const rclcpp::Time &now)
    {
        if (state == State::WAITING_FOR_START)
        {
            lap_count = 1;
            state = State::RECORDING_REFERENCE_LAP;
            current_lap_start_time = now;//set start time of the lap
            current_lap_sectors.clear();//clear the sectors vector in case there is any data
            RCLCPP_INFO(this->get_logger(), "Starting first lap!");
            return;
        }

        //Completed a lap
        lap_time = (now - current_lap_start_time).seconds();
        RCLCPP_INFO(this->get_logger(), "Lap %d finished! Time: %.3f s", lap_count, lap_time);

        if (state == State::RECORDING_REFERENCE_LAP)
        {
            // The first lap is automatically the best lap
            best_lap_time = lap_time;
            best_lap_sectors = current_lap_sectors;
            state = State::LAPPING;
            RCLCPP_INFO(this->get_logger(), "Reference map created. Switching to LAPPING mode.");
        }
        else if (state == State::LAPPING)
        {
            if (lap_time < best_lap_time)
            {
                best_lap_time = lap_time;
                best_lap_sectors = current_lap_sectors;
                RCLCPP_INFO(this->get_logger(), "New Best Lap! Time: %.3f s", best_lap_time);
            }
            else
            {
                // Slower lap
                double diff = lap_time - best_lap_time;
                RCLCPP_INFO(this->get_logger(), "Slower (+%.3f s)", diff);
            }
        }
        lap_count++;
        current_lap_start_time = now;
        current_lap_sectors.clear();
    }

    void record_current_sector(double current_lat, double current_lon, const rclcpp::Time &now)
    {
        // If there are no sectors yet, or the distance to the last sector is greater than DELTA_DISTANCE, record a new sector
        if (current_lap_sectors.empty() ||
            haversineDistance(current_lat, current_lon,
                              current_lap_sectors.back().lat,
                              current_lap_sectors.back().lon) >= DELTA_DISTANCE)
        {
            double time_s = (now - current_lap_start_time).seconds();
            current_lap_sectors.push_back({current_lat, current_lon, time_s});
            RCLCPP_INFO(this->get_logger(), "Recorded sector at (%.6f, %.6f) Time: %.3f s",
                        current_lat, current_lon, time_s);
        }
    }

    void calculate_delta(double current_lat, double current_lon, const rclcpp::Time &now)
    {
        if (best_lap_sectors.empty()) return;

        double min_dist = 10.0;
        int closest_idx = -1;

        // Find the closest point on the reference lap (Ghost Car position)
        for (size_t i = 0; i < m_best_lap_sectors.size(); ++i)
        {
            double d = haversineDistance(current_lat, current_lon, m_best_lap_sectors[i].lat, m_best_lap_sectors[i].lon);
            if (d < min_dist)
            {
                min_dist = d;
                closest_idx = i;
            }
        }

        // If we found a matching sector close enough (< 20m)
        if (closest_idx != -1 && min_dist < 20.0)
        {
            double current_time_s = (now - current_lap_start_time).seconds();//time into current lap
            double ghost_time_s = best_lap_sectors[closest_idx].time;
            
            // Delta = My Time - Ghost Time (Negative means faster)
            delta_time_s = current_time_s - ghost_time_s;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "Delta: %.3f | Curr: %.3f | Best: %.3f | Lap: %d",
                delta_time_s,      // Twoja delta
                current_time_s,    // Twój "lt" (obecny czas kółka)
                best_lap_time,     // Twój "blt" (najlepszy czas)
                lap_count          // Twój licznik okrążeń
            );
        }
    }

    void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        double current_lat = msg->latitude;
        double current_lon = msg->longitude;
        rclcpp::Time now = this->now(); 

        // 1. Check for Line Crossing (Runs in all states)
        handle_start_finish_crossing(current_lat, current_lon, now);

        // 2. State-Specific Behavior
        switch (m_state)
        {
        case State::WAITING_FOR_START:
            // Do nothing, just waiting for the green flag
            break;
            
        case State::RECORDING_REFERENCE_LAP:
            // Just record the map, no delta calculation yet
            record_current_sector(current_lat, current_lon, now);
            break;
            
        case State::LAPPING:
            // Record map AND calculate delta
            record_current_sector(current_lat, current_lon, now);
            calculate_delta(current_lat, current_lon, now);
            break;
        }
    }

    void lap_timer_callback()
    {
        // This function can be used to publish lap timer data periodically if needed
        // For now, it does nothing
    }


                



};

// Main function
int main(int argc, char **argv)
{
    // Initialize the ROS 2 node
    rclcpp::init(argc, argv);

    // Create a shared pointer to the LapTimer node
    auto node = std::make_shared<LapTimer>();

    // Spin the node
    rclcpp::spin(node);

    // Shutdown the ROS 2 node
    rclcpp::shutdown();

    return 0;
}
