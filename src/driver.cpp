//////////////////////////////////////////////////////////
// Novint Falcon ROS Driver.
//
// 2025 Michael Goerner
// based on code by Steven Martin
// based on barrow_mechanics example by Alistair Barrow

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Joy.h>

#include "falcon/core/FalconDevice.h"
#include "falcon/firmware/FalconFirmwareNovintSDK.h"
#include "falcon/util/FalconFirmwareBinaryNvent.h"
#include "falcon/grip/FalconGripFourButton.h"
// #include "falcon/util/FalconCLIBase.h"
// #include "falcon/kinematic/stamper/StamperUtils.h"
#include "falcon/kinematic/FalconKinematicStamper.h"
// #include "falcon/core/FalconGeometry.h"
// #include "falcon/gmtl/gmtl.h"

using namespace libnifalcon;
using namespace StamperKinematicImpl;

class Controller : public FalconDevice {
	ros::CallbackQueue cb_queue_;
	ros::NodeHandle nh_;
	ros::Publisher measured_cp_pub_;
	ros::Publisher joy_pub_;
	ros::Subscriber servo_cf_sub_;

	std::mutex force_mutex_;
	std::array<double, 3> cmd_force_ = {0.0, 0.0, 0.0};
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
		setFalconGrip<FalconGripFourButton>();

		nh_.setCallbackQueue(&cb_queue_);
		measured_cp_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("measured_cp", 1, true);
		joy_pub_ = nh_.advertise<sensor_msgs::Joy>("joy", 1, true);
		servo_cf_sub_ = nh_.subscribe<geometry_msgs::WrenchStamped>("servo_cf", 1, [=](auto&& msg) {
			servo_cf(*msg);
		});
	}

	~Controller() {
		close();
	}

	bool init(){
		if(!open(0)) {
			ROS_ERROR("Failed to connect to device");
			return false;
		}

		bool const skip_checksum = false;
		while(!isFirmwareLoaded()) {
			getFalconComm()->setFirmwareMode(); // this fails initially, but loadFirmware succeeds on first try this way
			if(getFalconFirmware()->loadFirmware(skip_checksum, NOVINT_FALCON_NVENT_FIRMWARE_SIZE, const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE)))
				break;
			ROS_ERROR_STREAM("Failed to upload firmware.");

			if(!ros::ok())
				return false;
		}

		return true;
	}

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

			runIOLoop(FALCON_LOOP_FIRMWARE);

			rate.sleep();
		}

		getFalconFirmware()->setHomingMode(false);
		ROS_INFO("Homing complete.");
		return true;
	}

	void servo_cf(const geometry_msgs::WrenchStamped& msg)
	{
		std::lock_guard<std::mutex> lock(force_mutex_);
		cmd_force_[0] = msg.wrench.force.x;
		cmd_force_[1] = msg.wrench.force.y;
		cmd_force_[2] = msg.wrench.force.z;
	}

	void spin(){
		runIOLoop();
		setForce({ 0, 0, 0 });
		getFalconFirmware()->setLEDStatus(FalconFirmware::GREEN_LED);
		runIOLoop();

		ros::AsyncSpinner spinner(1, &cb_queue_);
		spinner.start();

		geometry_msgs::PoseStamped p;
		p.pose.orientation.w = 1.0;
		p.header.frame_id = "falcon_base";

		sensor_msgs::Joy joy_msg;
		joy_msg.header.frame_id = "falcon_base";
		joy_msg.buttons.resize(getFalconGrip()->getNumDigitalInputs());

		std::array<double, 3> p_arr;

		ros::Rate rate{ 1000 };
		while(ros::ok()) {
			if(runIOLoop()) {
				auto const now{ ros::Time::now() };
				p_arr = getPosition();
				p.pose.position.x = p_arr[0];
				p.pose.position.y = p_arr[1];
				p.pose.position.z = p_arr[2];
				p.header.stamp = now;
				measured_cp_pub_.publish(p);

				joy_msg.header.stamp = now;
				joy_msg.axes.resize(3);
				joy_msg.axes[0] = p_arr[0];
				joy_msg.axes[1] = p_arr[1];
				joy_msg.axes[2] = p_arr[2];

				for(size_t i = 0; i < getFalconGrip()->getNumDigitalInputs(); ++i) {
					joy_msg.buttons[i] = getFalconGrip()->getDigitalInput(i);
				}
				joy_pub_.publish(joy_msg);
			}
			{
				std::lock_guard<std::mutex> lock(force_mutex_);
				setForce(cmd_force_);
			}
			rate.sleep();
		}

		spinner.stop();
	}
};

int main(int argc, char* argv[])
{
    ros::init(argc,argv, "falcon");
	ros::NodeHandle nh{"~"};

	ros::AsyncSpinner spinner(1);
	spinner.start();

	Controller dev{ nh };

	if(!dev.init())
		return 1;

	ROS_INFO("device initialized");

	if(!dev.home())
		return 1;

	dev.spin();

	return 0;
}

