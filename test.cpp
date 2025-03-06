#include <sick_safetyscanners_base/SickSafetyscanners.h>
#include <sick_safetyscanners_base/Exceptions.h>
#include <sick_safetyscanners_base/Types.h>
#include <sick_safetyscanners_base/datastructure/CommSettings.h>
#include "sick_safetyscanners_base/datastructure/ScanPoint.h"

#include <boost/asio.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>

int main() {
	// Sensor IP and Port
	std::string sensor_ip_str = ""; // IP COMMENTED OUT
	sick::types::ip_address_t sensor_ip = boost::asio::ip::address_v4::from_string(sensor_ip_str);
	sick::types::port_t tcp_port {2122};

	// Prepare the CommSettings for Sensor streaming data
	sick::datastructure::CommSettings comm_settings;
	std::string host_ip_str = "";  // Set to your host IP
	comm_settings.host_ip = boost::asio::ip::address_v4::from_string(host_ip_str);
	comm_settings.host_udp_port = 0;

	// Define a sensor data callback (working callback)
	// sick::types::ScanDataCb cb = [](const sick::datastructure::Data &data) {
	// 	std::cout << "Received data!" << std::endl;
	// 	std::cout << "Number of beams: " << data.getMeasurementDataPtr()->getNumberOfBeams() << std::endl;
	// 	// std::cout << "sensor status: " << std::endl;
	// 	sick::datastructure::MeasurementData &measurement_data;
	// 	std::vector<sick::datastructure::ScanPoint> scan_points = data.get
	// 	//std::vector<ScanPoint> lidar_data = data.getMeasurementDataPtr()->getScanPointsVector();
	// };

	// Define a sensor data callback
	sick::types::ScanDataCb cb = [](const sick::datastructure::Data &data) {
		std::cout << "Received data!" << std::endl;

		// number of beams (scan points)
		std::cout << "Number of beams: " << data.getMeasurementDataPtr()->getNumberOfBeams() << std::endl;

		// scan poitns
		std::vector<sick::datastructure::ScanPoint> scan_points = data.getMeasurementDataPtr()->getScanPointsVector();
		for (size_t i = 0; i < scan_points.size(); ++i) {
			const sick::datastructure::ScanPoint& point = scan_points[i];

			// scan point distance and angle
			std::cout << "Scan Point " << i << ": "
					<< "Distance: " << point.getDistance()/1000.0 << " meters, "
					<< "Angle: " << point.getAngle() << " degrees" << std::endl;
		}
	};

	// Create the sensor instance
	auto safety_scanner = std::make_unique<sick::AsyncSickSafetyScanner>(sensor_ip, tcp_port, comm_settings, cb);

	// Print initial status
	std::cout << "Attempting to connect to sensor at IP: " << sensor_ip_str << " on port: " << tcp_port << std::endl;

	try {
		// Start asynchronous receiving and processing of sensor data
		std::cout << "Starting asynchronous data reception..." << std::endl;
		safety_scanner->run();  // This starts the asynchronous data reception process

		// Simulating a main loop to keep the program running while the callback is triggered asynchronously
		while (true) {
			std::this_thread::sleep_for(std::chrono::seconds(1)); // Add sleep to prevent 100% CPU usage
			std::cout << "Waiting for sensor data..." << std::endl;

			// Check if you have received any error status or exit condition here
			// if (/* some exit condition */) {
			// 	std::cout << "Exiting data loop." << std::endl;
			// 	break;
			// }
		}

		std::cout << "Finished processing data." << std::endl;
	} catch (const std::exception &e) {
		std::cout << "Error occurred: " << e.what() << std::endl;
	}
}
