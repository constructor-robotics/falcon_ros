//////////////////////////////////////////////////////////
// Novint Falcon ROS Driver
//
// 2025 Michael Goerner
// based on code by Steven Martin
// based on barrow_mechanics example by Alistair Barrow

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_srvs/SetBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Joy.h>

#include "falcon/core/FalconDevice.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/grip/FalconGripFourButton.h"
#include "falcon/kinematic/FalconKinematicStamper.h"

#include <atomic>

using namespace libnifalcon;

class Controller : public FalconDevice {
	ros::NodeHandle nh_;
	ros::CallbackQueue cb_queue_;

	// TF spins independently to allow for commands in other frames
	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_{tf_buffer_, true};

	// base frame of all internal calculations
	std::string frame_id_;

	geometry_msgs::PoseStamped measured_cp_;
	ros::Publisher measured_cp_pub_;

	ros::Publisher joy_pub_;
	sensor_msgs::Joy joy_;

	ros::Subscriber servo_cf_sub_;

	std::atomic<bool> use_gravity_compensation_;
	ros::ServiceServer use_gravity_compensation_srv_;

	ros::Time now_; // time after last I/O loop

	std::array<double, 3> cmd_force_ {0.0, 0.0, 0.0};
	std::mutex cmd_force_mutex_;
public:
	Controller(ros::NodeHandle nh) :
		FalconDevice(),
		nh_(nh)
	{
		// set only communication protocol (matching the firmware loaded later)
		setFalconFirmware<FalconFirmwareNovintSDK>();
		getFalconFirmware()->setForces(std::array<int, 3>{ 0, 0, 0 });

		// set only kinematic model
		setFalconKinematic<libnifalcon::FalconKinematicStamper>();

		// set standard 4-button grip (again, the only grip directly available in the library)
		setFalconGrip<FalconGripFourButton>();

		frame_id_ = nh_.param<std::string>("frame_id", "falcon_front");

		// prepare published data
		joy_.header.frame_id = frame_id_;
		joy_.axes.resize(3);
		joy_.buttons.resize(getFalconGrip()->getNumDigitalInputs());
		measured_cp_.header.frame_id = frame_id_;
		measured_cp_.pose.orientation.w = 1.0;

		// ROS communication
		nh_.setCallbackQueue(&cb_queue_);

		measured_cp_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("measured_cp", 1, true);
		joy_pub_ = nh_.advertise<sensor_msgs::Joy>("joy", 1, true);

		use_gravity_compensation(true);
		use_gravity_compensation_srv_ = nh_.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(
			"use_gravity_compensation",
			[this](auto&& req, auto&& res) {
				use_gravity_compensation(req.data);
				res.success = true;
				return true;
			}
		);

		servo_cf_sub_ = nh_.subscribe<geometry_msgs::WrenchStamped>("servo_cf", 1, [=](auto&& msg) {
			servo_cf(*msg);
		});
	}

	~Controller() {
		getFalconFirmware()->setLEDStatus(FalconFirmware::BLUE_LED | FalconFirmware::RED_LED | FalconFirmware::GREEN_LED);
		runIOLoop(FALCON_LOOP_FIRMWARE);
		close();
	}

	// open device and load firmware
	bool init(){
		if(!open(0)) {
			ROS_ERROR("Failed to connect to device");
			return false;
		}

		bool const skip_checksum = false;

		// running this check with I/O loops when no firmware is loaded breaks behavior after following upload, so we do not do it.
		// if (isFirmwareLoaded())
		// 	return true;

		if(!getFalconFirmware()->loadFirmware(skip_checksum, NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE))){
			ROS_DEBUG_STREAM("Attempt to load firmware failed. error code: " << getFalconFirmware()->getErrorCode() << " (device error code: " << getFalconComm()->getDeviceErrorCode() << ")");
			close();
			return false;
		}

		ROS_DEBUG("Firmware loaded");

		return true;
	}

	bool initialized() {
		return isOpen() && isFirmwareLoaded();
	}

	// block until device is homed (all joint stops were detected)
	bool home(){
		runIOLoop(FALCON_LOOP_FIRMWARE);
		if(getFalconFirmware()->isHomed()) {
			ROS_INFO("device is already homed.");
			return true;
		}

		getFalconFirmware()->setHomingMode(true);
		getFalconFirmware()->setLEDStatus(FalconFirmware::BLUE_LED);
		runIOLoop(FALCON_LOOP_FIRMWARE);

		if(!getFalconFirmware()->isHomed())
			ROS_INFO("Falcon not currently homed. Move control all the way out then push straight all the way in.");

		ros::Rate rate{ 100 };
		while(!getFalconFirmware()->isHomed())
		{
			if (!ros::ok())
				return false;

			runIOLoop();

			rate.sleep();
		}

		getFalconFirmware()->setHomingMode(false);
		ROS_INFO("Homing complete.");
		return true;
	}

	void use_gravity_compensation(bool use){
		ROS_INFO_STREAM("Gravity compensation " << (use ? "active" : "disabled") << ".");
		use_gravity_compensation_ = use;
	}

	// CRTK: update Cartesian force command
	void servo_cf(geometry_msgs::WrenchStamped msg)
	{
		if(!msg.header.frame_id.empty() && msg.header.frame_id != frame_id_) {
			geometry_msgs::TransformStamped transform;
			try {
				// assume static transform for force vector because this code should never wait
				transform = tf_buffer_.lookupTransform(frame_id_, msg.header.frame_id, ros::Time(0));
				// the Falcon cannnot apply torque
				tf2::doTransform(msg.wrench.force, msg.wrench.force, transform);
			} catch (tf2::TransformException& ex) {
				ROS_WARN_STREAM_THROTTLE(0.1, "Failed to transform from " << msg.header.frame_id << " to " << frame_id_ << ": " << ex.what());
				return;
			}
		}

		std::lock_guard<std::mutex> lock{ cmd_force_mutex_ };
		cmd_force_[0] = msg.wrench.force.x;
		cmd_force_[1] = msg.wrench.force.y;
		cmd_force_[2] = msg.wrench.force.z;
	}

	// CRTK: publish Cartesian pose
	void measured_cp(){
		auto const& p_arr = getPosition();
		measured_cp_.pose.position.x = p_arr[0];
		measured_cp_.pose.position.y = p_arr[1];
		measured_cp_.pose.position.z = p_arr[2];
		measured_cp_.header.stamp = now_;
		measured_cp_pub_.publish(measured_cp_);
	}

	// publish joystick message
	void joy(){
		auto const& p_arr = getPosition();
		joy_.header.stamp = now_;
		joy_.axes[0] = p_arr[0];
		joy_.axes[1] = p_arr[1];
		joy_.axes[2] = p_arr[2];
		for(size_t i = 0; i < getFalconGrip()->getNumDigitalInputs(); ++i) {
			joy_.buttons[i] = getFalconGrip()->getDigitalInput(i);
		}
		joy_pub_.publish(joy_);
	}

	// set Cartesian force command for next I/O loop
	void cmd_cf(){
		std::array<double, 3> cmd{
			[&]{
				std::lock_guard<std::mutex> lock{ cmd_force_mutex_ };
				return cmd_force_;
			}()
		 };

		if(use_gravity_compensation_) {
			double const g = 9.81; // m/s^2
			double const effective_mass = 0.090; // kg, determined through controller performance
			cmd[1] += g * effective_mass; // y points up in Falcon coordinate system
		}
		setForce(cmd);
	}

	void spin(){
		ros::AsyncSpinner spinner{ 2, &cb_queue_ };
		spinner.start();

		setForce({ 0, 0, 0 });
		getFalconFirmware()->setLEDStatus(FalconFirmware::GREEN_LED);
		runIOLoop();

		ros::Rate rate{ 1000 };
		while(ros::ok()) {
			if(runIOLoop()) {
				now_ = ros::Time::now();

				measured_cp();
				joy();
			}

			cmd_cf();
			rate.sleep();
		}

		spinner.stop();
	}
};

int main(int argc, char* argv[])
{
	ros::init(argc,argv, "falcon");
	ros::NodeHandle nh{"~"};

	Controller dev{ nh };

	{
		for(size_t i = 0; i < 3; ++i){
			if(dev.init())
				break;
		}
	}

	if(!dev.initialized()) {
		ROS_ERROR("Failed to initialize Falcon.");
		return 1;
	}

	ROS_INFO("Falcon initialized.");

	if(!dev.home())
		return 1;

	dev.spin();

	return 0;
}
