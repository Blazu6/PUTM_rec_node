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
    LapTimer() : Node("lap_timer"), m_state(State::WAITING_FOR_START){
        // Create a subscription to the "/vectornav/gnss" topic with a QoS of 50
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/vectornav/gnss", 50, std::bind(&LapTimer::gps_callback, this, std::placeholders::_1));
            
        // Create publishers for the lap timer delta and time
        // Uncomment line below when using on a car with putm_vcl_interfaces
        // lap_timer_pub = this->create_publisher<putm_vcl_interfaces::msg::LapTimer>("/putm_vcl/lap_timer", 50);
        
        // Create a timer that triggers every 20 milliseconds
        timer_ = this->create_wall_timer(
            20ms, std::bind(&LapTimer::lap_timer_callback, this));

        // Create a subscription to the "/vectornav/velocity_body" topic with a QoS of 10. This is to fetch the current speed of the vehicle.
        body_velocity_sub= this->create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/vectornav/velocity_body", 10, std::bind(&LapTimer::vel_bd_callback, this, std::placeholders::_1));
        // //create csv file and trigger a header
        // file_.open("/home/franiuu/PUTM_VP_LAPTIMER/log/laptimer_data_lap0.csv");
        // if(!file_.is_open()) {
        // RCLCPP_ERROR(this->get_logger(), "Could not open lap_timer_data.csv");
        // }

        // file_ << std::fixed << std::setprecision(10);
        // file_ << "current checkpoint,delta,current latitude,current longitude,lap_count,current_speed\n";
        // file_.flush();

        RCLCPP_INFO(this->get_logger(), "LapTimer Initialized. State: WAITING_FOR_START");
    
    }
private:

    bool is_approaching_start = false;
    double closest_approach = 100.0;
    bool has_crossed_this_pass = false;

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
