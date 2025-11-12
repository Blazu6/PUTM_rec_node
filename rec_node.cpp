#include <rec_node/rec_node.hpp>

using namespace putm_vcl_interfaces;
using namespace std::chrono_literals;
using std::placeholders::_1;

RecNode::RecNode()
    : Node("rec_node"),
    rtd_subscription(this->create_subscription<msg::Rtd>("rtd", 1, std::bind(&RecNode::rtd_callback, this, _1))),
    recording(false),pid_(-1) {}

void RecNode::rtd_callback(const msg::Rtd::SharedPtr msg) {
if (msg->state && !recording) {
    RCLCPP_INFO(this->get_logger(), "Car is ready. Starting data recording.");
    start_recording();
    if (stop_timer_)
    {
        stop_timer_->cancel();
        stop_timer_.reset();
    }
} else if (!msg->state && recording && !stop_timer_) {
        RCLCPP_INFO(this->get_logger(), "Car is not ready. Stopping data recording after 20 seconds.");
        stop_timer_ = this->create_wall_timer(
            20s,
            [this]() 
            {
                if (recording) {
                    RCLCPP_INFO(this->get_logger(), "20 seconds passed. Stopping recording.");
                    stop_recording();
                }
                stop_timer_.reset();
            }
        );
}
}

void RecNode::start_recording() {
    // Domyślny prefix
    std::string prefix = "recording";

    // 1. Wczytanie słowa z pliku konfiguracyjnego
    std::ifstream config_file("/home/putm/rosbag.config");
    if (config_file.is_open()) {
        std::string file_word;
        config_file >> file_word; // pobiera pierwsze słowo z pliku
        if (!file_word.empty()) {
            prefix = file_word; // nadpisujemy prefix tylko jeśli plik nie jest pusty
        }
        config_file.close();
    } else {
        std::cerr << "Nie udało się otworzyć pliku konfiguracyjnego. Używam domyślnej nazwy.\n";
    }

    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&time_t_now), "%Y-%m-%d_%H-%M-%S");
    
    std::string filename = "/home/putm/rosbag/" + prefix + "_" + oss.str(); // Path to the directory where the recordings will be saved
    
    pid_ = fork();
    if (pid_ == -1) {
        RCLCPP_ERROR(this->get_logger(), "Error while creating the process!");
        return;
    }

    if (pid_ == 0) {
        execlp("ros2", "ros2", "bag", "record", "-a", "-s", "mcap", "-o", filename.c_str(), nullptr);
        RCLCPP_ERROR(this->get_logger(), "Failed to launch `ros2 bag record`!");
        _exit(1);
    } else {
        RCLCPP_INFO(this->get_logger(), "Started recording with PID: %d", pid_);
        recording = true;
    }

}

void RecNode::stop_recording() {
    if (recording) {
        if (pid_ > 0) {
            RCLCPP_INFO(this->get_logger(), "Stopping procces number (PID: %d)...", pid_);
            kill(pid_, SIGTERM);

            int status;
            waitpid(pid_, &status, 0);
            if (WIFEXITED(status)) {
                RCLCPP_INFO(this->get_logger(), "Procces ros2 bag record completed correctly.");
            } else {
                RCLCPP_WARN(this->get_logger(), "Procces ros2 bag record completed incorrectly.");
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Incorrect PID of the process ros2 bag record.");
        }
        recording = false;
    }
}


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RecNode>());
    rclcpp::shutdown();
    return 0;
}
