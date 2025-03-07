// #include <sick_safetyscanners_base/SickSafetyscanners.h>
// #include <sick_safetyscanners_base/Exceptions.h>
// #include <sick_safetyscanners_base/Types.h>
// #include <sick_safetyscanners_base/datastructure/CommSettings.h>

// #include <opencv2/opencv.hpp>

// #include <boost/asio.hpp>
// #include <iostream>
// #include <chrono>
// #include <thread>
// #include <vector>
// #include <cmath>

// int main() {
// 	// Sensor IP and Port
// 	std::string sensor_ip_str = "192.168.1.3"; // IP COMMENTED OUT
// 	sick::types::ip_address_t sensor_ip = boost::asio::ip::address_v4::from_string(sensor_ip_str);
// 	sick::types::port_t tcp_port {2122};

// 	// Prepare the CommSettings for Sensor streaming data
// 	sick::datastructure::CommSettings comm_settings;
// 	std::string host_ip_str = "192.168.1.2";  // Set to your host IP
// 	comm_settings.host_ip = boost::asio::ip::address_v4::from_string(host_ip_str);
// 	comm_settings.host_udp_port = 0;

// 	// Define a sensor data callback (working callback)
// 	// sick::types::ScanDataCb cb = [](const sick::datastructure::Data &data) {
// 	// 	std::cout << "Received data!" << std::endl;
// 	// 	std::cout << "Number of beams: " << data.getMeasurementDataPtr()->getNumberOfBeams() << std::endl;
// 	// 	// std::cout << "sensor status: " << std::endl;
// 	// 	sick::datastructure::MeasurementData &measurement_data;
// 	// 	std::vector<sick::datastructure::ScanPoint> scan_points = data.get
// 	// 	//std::vector<ScanPoint> lidar_data = data.getMeasurementDataPtr()->getScanPointsVector();
// 	// };

// 	// Define a sensor data callback
// 	sick::types::ScanDataCb cb = [](const sick::datastructure::Data &data) {
// 		std::cout << "Received data!" << std::endl;
// 		cv::namedWindow("ldiar real time plot", cv::WINDOW_AUTOSIZE);
// 		// number of beams (scan points)
// 		std::cout << "Number of beams: " << data.getMeasurementDataPtr()->getNumberOfBeams() << std::endl;

// 		// scan poitns
// 		std::vector<sick::datastructure::ScanPoint> scan_points = data.getMeasurementDataPtr()->getScanPointsVector();
// 		for (size_t i = 0; i < scan_points.size(); ++i) {
// 			const sick::datastructure::ScanPoint& point = scan_points[i];

// 			// scan point distance and angle
// 			std::cout << "Scan Point " << i << ": "
// 					<< "Distance: " << point.getDistance()/1000.0 << " meters, "
// 					<< "Angle: " << point.getAngle() << " degrees" << std::endl;
// 		}
// 	};

// 	// Create the sensor instance
// 	auto safety_scanner = std::make_unique<sick::AsyncSickSafetyScanner>(sensor_ip, tcp_port, comm_settings, cb);

// 	// Print initial status
// 	std::cout << "Attempting to connect to sensor at IP: " << sensor_ip_str << " on port: " << tcp_port << std::endl;

// 	try {
// 		// Start asynchronous receiving and processing of sensor data
// 		std::cout << "Starting asynchronous data reception..." << std::endl;
// 		safety_scanner->run();  // This starts the asynchronous data reception process

// 		// Simulating a main loop to keep the program running while the callback is triggered asynchronously
// 		while (true) {
// 			std::this_thread::sleep_for(std::chrono::seconds(1)); // Add sleep to prevent 100% CPU usage
// 			std::cout << "Waiting for sensor data..." << std::endl;

// 			// Check if you have received any error status or exit condition here
// 			// if (/* some exit condition */) {
// 			// 	std::cout << "Exiting data loop." << std::endl;
// 			// 	break;
// 			// }
// 		}

// 		std::cout << "Finished processing data." << std::endl;
// 	} catch (const std::exception &e) {
// 		std::cout << "Error occurred: " << e.what() << std::endl;
// 	}
// }






// working




// #include <sick_safetyscanners_base/SickSafetyscanners.h>
// #include <sick_safetyscanners_base/Exceptions.h>
// #include <sick_safetyscanners_base/Types.h>
// #include <sick_safetyscanners_base/datastructure/CommSettings.h>

// #include <opencv2/opencv.hpp>

// #include <boost/asio.hpp>
// #include <iostream>
// #include <chrono>
// #include <thread>
// #include <vector>
// #include <cmath>
// #include <mutex>


// // Thread-safe storage for LiDAR data
// std::vector<sick::datastructure::ScanPoint> g_scan_points;
// std::mutex g_data_mutex;

// int main() {
//     // Sensor IP and Port
//     std::string sensor_ip_str = "192.168.1.3"; // IP COMMENTED OUT
//     sick::types::ip_address_t sensor_ip = boost::asio::ip::address_v4::from_string(sensor_ip_str);
//     sick::types::port_t tcp_port {2122};

//     // Prepare the CommSettings for Sensor streaming data
//     sick::datastructure::CommSettings comm_settings;
//     std::string host_ip_str = "192.168.1.2";  // Set to your host IP
//     comm_settings.host_ip = boost::asio::ip::address_v4::from_string(host_ip_str);
//     comm_settings.host_udp_port = 0;

//     // Create OpenCV window
//     cv::namedWindow("LiDAR Real-Time Plot", cv::WINDOW_AUTOSIZE);
//     cv::Mat display(600, 800, CV_8UC3, cv::Scalar(255, 255, 255)); // Black background

//     // Define a sensor data callback
//     sick::types::ScanDataCb cb = [](const sick::datastructure::Data &data) {
//         std::lock_guard<std::mutex> lock(g_data_mutex);
//         g_scan_points = data.getMeasurementDataPtr()->getScanPointsVector();
//     };

//     // Create the sensor instance
//     auto safety_scanner = std::make_unique<sick::AsyncSickSafetyScanner>(sensor_ip, tcp_port, comm_settings, cb);

//     // Print initial status
//     std::cout << "Attempting to connect to sensor at IP: " << sensor_ip_str << " on port: " << tcp_port << std::endl;

//     try {
//         // Start asynchronous data reception
//         safety_scanner->run();

//         // Main rendering loop
//         while (true) {
//             // Get latest scan points
//             std::vector<sick::datastructure::ScanPoint> scan_points;
//             {
//                 std::lock_guard<std::mutex> lock(g_data_mutex);
//                 scan_points = g_scan_points;
//             }

//             // Convert to Cartesian coordinates and draw
//             display.setTo(cv::Scalar(255, 255, 255)); // Clear screen

// 			// Draw scanner position (a small black circle at the center)
// 			cv::Point scanner_pos(400, 300);  // Center of the window
// 			int scanner_radius = 5;           // Size of the scanner indicator
// 			cv::circle(display, scanner_pos, scanner_radius, cv::Scalar(0, 0, 0), 3); // Filled circle


//             for (const auto& point : scan_points) {
//                 double angle_rad = point.getAngle() * M_PI / 180.0; // Convert degrees to radians
//                 double distance_m = point.getDistance() / 1000.0;    // Convert mm to meters

//                 // Convert to pixel coordinates (scale: 50 pixels = 1 meter)
//                 int x = static_cast<int>(distance_m * 50 * std::cos(angle_rad)) + 400; // Center at (400,300)
//                 int y = static_cast<int>(distance_m * 50 * std::sin(angle_rad)) + 300;

//                 // Draw red dots
//                 cv::circle(display, cv::Point(x, y), 1, cv::Scalar(0, 0, 255), -1);
//             }

//             // Show the plot
//             cv::imshow("LiDAR Real-Time Plot", display);
//             if (cv::waitKey(30) == 27) break; // Exit on ESC key

//             std::this_thread::sleep_for(std::chrono::milliseconds(30)); // ~30 FPS
//         }

//         std::cout << "Finished processing data." << std::endl;
//     } catch (const std::exception &e) {
//         std::cout << "Error occurred: " << e.what() << std::endl;
//     }

//     return 0;
// }






// #include <sick_safetyscanners_base/SickSafetyscanners.h>
// #include <sick_safetyscanners_base/Exceptions.h>
// #include <sick_safetyscanners_base/Types.h>
// #include <sick_safetyscanners_base/datastructure/CommSettings.h>
// #include <sick_safetyscanners_base/datastructure/IntrusionData.h> // Include for field data

// #include <opencv2/opencv.hpp>

// #include <boost/asio.hpp>
// #include <iostream>
// #include <chrono>
// #include <thread>
// #include <vector>
// #include <cmath>
// #include <mutex>

// // Thread-safe storage
// std::vector<sick::datastructure::ScanPoint> g_scan_points;
// std::mutex g_data_mutex;

// int main() {
//     // Sensor IP and Port
//     std::string sensor_ip_str = "192.168.1.3";
//     sick::types::ip_address_t sensor_ip = boost::asio::ip::address_v4::from_string(sensor_ip_str);
//     sick::types::port_t tcp_port {2122};

//     // Prepare CommSettings
//     sick::datastructure::CommSettings comm_settings;
//     comm_settings.host_ip = boost::asio::ip::address_v4::from_string("192.168.1.2");
//     comm_settings.host_udp_port = 0;

//     // OpenCV setup
//     cv::namedWindow("LiDAR Real-Time Plot", cv::WINDOW_AUTOSIZE);
//     cv::Mat display(600, 800, CV_8UC3, cv::Scalar(255, 255, 255));

//     // Callback to process sensor data
//     sick::types::ScanDataCb cb = [](const sick::datastructure::Data &data) {
//         std::lock_guard<std::mutex> lock(g_data_mutex);

//         // Update scan points
//         g_scan_points = data.getMeasurementDataPtr()->getScanPointsVector();

//         // Extract intrusion data vector
//         auto intrusion_data_vector = data.getIntrusionDataPtr()->getIntrusionDataVector();
//         if (!intrusion_data_vector.empty()) {
//             std::cout << "Intrusion Data Vector Size: " << intrusion_data_vector.size() << std::endl;
//             for (size_t i = 0; i < intrusion_data_vector.size(); ++i) {
//                 const auto& intrusion_data = intrusion_data_vector[i];
// 				std::cout << "Intrusion Data Entry #" << i + 1 << ":" << std::endl;

//             }
//         } else {
//             std::cout << "Intrusion Data Vector is Empty" << std::endl;
//         }
//     };

//     // Initialize sensor
//     auto safety_scanner = std::make_unique<sick::AsyncSickSafetyScanner>(sensor_ip, tcp_port, comm_settings, cb);

//     try {
//         safety_scanner->run();

//         // Main rendering loop
//         while (true) {
//             std::vector<sick::datastructure::ScanPoint> scan_points;
//             {
//                 std::lock_guard<std::mutex> lock(g_data_mutex);
//                 scan_points = g_scan_points;
//             }

//             // Clear screen
//             display.setTo(cv::Scalar(255, 255, 255));

//             // Draw scanner position (black circle)
//             cv::circle(display, cv::Point(400, 300), 5, cv::Scalar(0, 0, 0), -1);

//             // Draw LiDAR points (red)
//             for (const auto& point : scan_points) {
//                 double angle_rad = point.getAngle() * M_PI / 180.0;
//                 double distance_m = point.getDistance() / 1000.0;

//                 int x = static_cast<int>(distance_m * 50 * std::cos(angle_rad)) + 400;
//                 int y = static_cast<int>(distance_m * 50 * std::sin(angle_rad)) + 300;
//                 cv::circle(display, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1);
//             }

//             // Show plot
//             cv::imshow("LiDAR Real-Time Plot", display);
//             if (cv::waitKey(30) == 27) break;
//         }
//     } catch (const std::exception &e) {
//         std::cerr << "Error: " << e.what() << std::endl;
//     }

//     return 0;
// }

// #include <sick_safetyscanners_base/SickSafetyscanners.h>
// #include <sick_safetyscanners_base/Exceptions.h>
// #include <sick_safetyscanners_base/Types.h>
// #include <sick_safetyscanners_base/datastructure/CommSettings.h>
// #include <sick_safetyscanners_base/datastructure/IntrusionData.h> // Include for field data

// #include <opencv2/opencv.hpp>

// #include <boost/asio.hpp>
// #include <iostream>
// #include <chrono>
// #include <thread>
// #include <vector>
// #include <cmath>
// #include <mutex>

// // Thread-safe storage
// std::vector<sick::datastructure::ScanPoint> g_scan_points;
// std::vector<int> g_interrupted_beams; // Store indices of interrupted beams
// std::mutex g_data_mutex;


// int main() {
//     // Sensor IP and Port
//     std::string sensor_ip_str = "192.168.1.3";
//     sick::types::ip_address_t sensor_ip = boost::asio::ip::address_v4::from_string(sensor_ip_str);
//     sick::types::port_t tcp_port{2122};

//     // Prepare CommSettings
//     sick::datastructure::CommSettings comm_settings;
//     comm_settings.host_ip = boost::asio::ip::address_v4::from_string("192.168.1.2");
//     comm_settings.host_udp_port = 0;

//     // OpenCV setup
//     cv::namedWindow("LiDAR Real-Time Plot", cv::WINDOW_AUTOSIZE);
//     cv::Mat display(600, 800, CV_8UC3, cv::Scalar(255, 255, 255));

//     // Callback to process sensor data
//     sick::types::ScanDataCb cb = [](const sick::datastructure::Data &data) {
//         std::lock_guard<std::mutex> lock(g_data_mutex);

//         // Update scan points
//         g_scan_points = data.getMeasurementDataPtr()->getScanPointsVector();

//         // Clear previous interrupted beams
//         g_interrupted_beams.clear();

// 		bool sensor_status = data.getGeneralSystemStatePtr()->getCurrentMonitoringCaseNoTable1();
// 		if (sensor_status) {
// 			std::cout << "[Sensor Status]: Good" << std::endl;
// 		}
// 		else {
// 			std::cout << "[Sensor Status]: Bad" << std::endl;
// 		}

//         // Extract intrusion data
//         auto intrusion_data_vector = data.getIntrusionDataPtr()->getIntrusionDataVector();
// 		if (!intrusion_data_vector.empty()) {
// 			for (const auto &intrusion_data : intrusion_data_vector) {
// 				// Get the flags vector (assuming getFlagsVector() exists)
// 				auto flags = intrusion_data.getFlagsVector();

// 				// Parse the flags vector
// 				for (size_t byte_idx = 0; byte_idx < flags.size(); ++byte_idx) {
// 					uint8_t flag_byte = flags[byte_idx]; // Each byte contains 8 bits (beams)

// 					for (int bit = 0; bit < 8; ++bit) {
// 						if (flag_byte & (1 << bit)) {
// 							// Calculate the global beam index
// 							size_t beam_idx = byte_idx * 8 + bit;

// 							// Add the interrupted beam index to the list
// 							g_interrupted_beams.push_back(beam_idx);

// 							// Print the interrupted beam index
// 							std::cout << "Interrupted beam detected: " << beam_idx << std::endl;
// 						}
// 					}
// 				}
// 			}
// 		}
        
//     };

//     // Initialize sensor
//     auto safety_scanner = std::make_unique<sick::AsyncSickSafetyScanner>(sensor_ip, tcp_port, comm_settings, cb);

//     try {
//         safety_scanner->run();

//         // Main rendering loop
//         while (true) {
//             std::vector<sick::datastructure::ScanPoint> scan_points;
//             std::vector<int> interrupted_beams;
//             {
//                 std::lock_guard<std::mutex> lock(g_data_mutex);
//                 scan_points = g_scan_points;
//                 interrupted_beams = g_interrupted_beams;
//             }

//             // Clear screen
//             display.setTo(cv::Scalar(255, 255, 255));

//             // Draw scanner position (black circle)
//             cv::circle(display, cv::Point(400, 300), 5, cv::Scalar(0, 0, 0), 2);

//             // Draw LiDAR points
//             for (size_t i = 0; i < scan_points.size(); ++i) {
//                 const auto &point = scan_points[i];
//                 double angle_rad = point.getAngle() * M_PI / 180.0;
//                 double distance_m = point.getDistance() / 1000.0;

//                 int x = static_cast<int>(distance_m * 50 * std::cos(angle_rad)) + 400;
//                 int y = static_cast<int>(distance_m * 50 * std::sin(angle_rad)) + 300;

//                 // Check if the beam is interrupted
//                 bool is_interrupted = false;
//                 for (int interrupted_beam : interrupted_beams) {
//                     if (static_cast<int>(i) == interrupted_beam) {
//                         is_interrupted = true;
//                         break;
//                     }
//                 }

//                 // Draw blue for interrupted beams, red otherwise
//                 if (is_interrupted) {
//                     cv::circle(display, cv::Point(x, y), 2, cv::Scalar(255, 0, 0), -1); // Blue
//                 } else {
//                     cv::circle(display, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1); // Red
//                 }
//             }

//             // Show plot
//             cv::imshow("LiDAR Real-Time Plot", display);
//             if (cv::waitKey(30) == 27) break;
//         }
//     } catch (const std::exception &e) {
//         std::cerr << "Error: " << e.what() << std::endl;
//     }

//     return 0;
// }



#include <sick_safetyscanners_base/SickSafetyscanners.h>
#include <sick_safetyscanners_base/Exceptions.h>
#include <sick_safetyscanners_base/Types.h>
#include <sick_safetyscanners_base/datastructure/CommSettings.h>
#include <sick_safetyscanners_base/datastructure/IntrusionData.h>
#include <opencv2/opencv.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <mutex>
#include <typeinfo>

std::vector<sick::datastructure::ScanPoint> g_scan_points;
std::vector<int> g_interrupted_beams;
std::mutex g_data_mutex;

// void printIntrusionDataVector(const sick::datastructure::Data &data) {
// 	// auto intrusion_data_vector = data.getIntrusionDataPtr()->getIntrusionDataVector();
// 	std::vector<IntrusionDatum> intrusion_data_vector = data.getIntrusionDataPtr()->getIntrusionDataVector();
//     // Retrieve the monitoring case flags vector
//     //auto flags = general_state->getIntrusionDataVector();
	
//     // for (auto i = 0; i < idk.size(); ++i) {
//     //     std::cout << "yep: " << idk[i] << std::endl;
//     // }
// }

void printMonitoringCaseFlags(const sick::datastructure::Data &data) {
    auto general_state = data.getApplicationDataPtr();
    if (!general_state) {
        std::cerr << "Error: Unable to retrieve general system state!" << std::endl;
        return;
    }

    // Get Application Inputs
    auto inputs = general_state->getInputs();

    // Retrieve the monitoring case flags vector
    std::vector<uint16_t> flags = inputs.getMonitoringCasevector();

    std::cout << "Monitoring Case Flags: ";
    for (size_t i = 0; i < flags.size(); ++i) {
        std::cout << "yep:" << i << " " << flags[i] << std::endl;
    }
    std::cout << std::endl;
}

void printSensorGeneralInfo(const sick::datastructure::Data &data) {
    std::cout << "---- Sensor General Information ----" << std::endl;

    std::cout << "isEmpty: " << data.getGeneralSystemStatePtr()->isEmpty() << std::endl;
    std::cout << "Get Run Mode Active: " << data.getGeneralSystemStatePtr()->getRunModeActive() << std::endl;
    std::cout << "getStandbyModeActive: " << data.getGeneralSystemStatePtr()->getStandbyModeActive() << std::endl;
    std::cout << "getContaminationWarning: " << data.getGeneralSystemStatePtr()->getContaminationWarning() << std::endl;
    std::cout << "getContaminationError: " << data.getGeneralSystemStatePtr()->getContaminationError() << std::endl;
    std::cout << "getApplicationError: " << data.getGeneralSystemStatePtr()->getApplicationError() << std::endl;
    std::cout << "getDeviceError: " << data.getGeneralSystemStatePtr()->getDeviceError() << std::endl;
    std::cout << "getSerialNumberOfDevice: " << data.getDataHeaderPtr()->getSerialNumberOfDevice() << std::endl;
	std::cout << "getMultiplicationFactor: " << data.getDerivedValuesPtr()->getMultiplicationFactor() << std::endl;
	
	std::cout << "------------------------------------" << std::endl;
}

void processSensorData(const sick::datastructure::Data &data) {
    std::lock_guard<std::mutex> lock(g_data_mutex);
    g_scan_points = data.getMeasurementDataPtr()->getScanPointsVector();
    g_interrupted_beams.clear();

	// print metadata
	printSensorGeneralInfo(data);
	printMonitoringCaseFlags(data);
	// printIntrusionDataVector(data);

    bool sensor_status = data.getGeneralSystemStatePtr()->getCurrentMonitoringCaseNoTable1();
    std::cout << "[Sensor Status]: " << (sensor_status ? "Good" : "Bad") << std::endl;

    auto intrusion_data_vector = data.getIntrusionDataPtr()->getIntrusionDataVector();
    for (const auto &intrusion_data : intrusion_data_vector) {
        auto flags = intrusion_data.getFlagsVector();
        for (size_t byte_idx = 0; byte_idx < flags.size(); ++byte_idx) {
            uint8_t flag_byte = flags[byte_idx];
            for (int bit = 0; bit < 8; ++bit) {
                if (flag_byte & (1 << bit)) {
                    g_interrupted_beams.push_back(byte_idx * 8 + bit);
                    //std::cout << "Interrupted beam detected: " << (byte_idx * 8 + bit) << std::endl;
                }
            }
        }
    }
}

void drawLidarData(cv::Mat &display) {
    display.setTo(cv::Scalar(255, 255, 255));
    cv::circle(display, cv::Point(400, 300), 5, cv::Scalar(0, 0, 0), 2);

    std::vector<sick::datastructure::ScanPoint> scan_points;
    std::vector<int> interrupted_beams;
    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        scan_points = g_scan_points;
        interrupted_beams = g_interrupted_beams;
    }

    for (size_t i = 0; i < scan_points.size(); ++i) {
        const auto &point = scan_points[i];
        double angle_rad = point.getAngle() * M_PI / 180.0;
        double distance_m = point.getDistance() / 1000.0;
        int x = static_cast<int>(distance_m * 50 * std::cos(angle_rad)) + 400;
        int y = static_cast<int>(distance_m * 50 * std::sin(angle_rad)) + 300;

        bool is_interrupted = (std::find(interrupted_beams.begin(), interrupted_beams.end(), i) != interrupted_beams.end());
        cv::circle(display, cv::Point(x, y), 2, is_interrupted ? cv::Scalar(255, 0, 0) : cv::Scalar(0, 0, 255), -1);
    }
    cv::imshow("LiDAR Real-Time Plot", display);
}

int main() {
    std::string sensor_ip_str = "192.168.1.3";
    sick::types::ip_address_t sensor_ip = boost::asio::ip::address_v4::from_string(sensor_ip_str);
    sick::types::port_t tcp_port{2122};
    
    sick::datastructure::CommSettings comm_settings;
    comm_settings.host_ip = boost::asio::ip::address_v4::from_string("192.168.1.2");
    comm_settings.host_udp_port = 0;

    cv::namedWindow("LiDAR Real-Time Plot", cv::WINDOW_AUTOSIZE);
    cv::Mat display(600, 800, CV_8UC3, cv::Scalar(255, 255, 255));

    sick::types::ScanDataCb cb = processSensorData;
    auto safety_scanner = std::make_unique<sick::AsyncSickSafetyScanner>(sensor_ip, tcp_port, comm_settings, cb);
    
    try {
        safety_scanner->run();
        while (true) {
            drawLidarData(display);
            if (cv::waitKey(30) == 27) break;
        }
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    return 0;
}
