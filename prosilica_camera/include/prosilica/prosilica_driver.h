/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "prosilica/rolling_sum.h"

namespace prosilica_driver
{
/// @todo Only stream when subscribed to
class ProsilicaDriver
{
private:
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::CameraPublisher streaming_pub_;
	polled_camera::PublicationServer poll_srv_;
	ros::ServiceServer set_camera_info_srv_;
	ros::Subscriber trigger_sub_;

	// Camera
	boost::scoped_ptr<prosilica::Camera> cam_;
	prosilica::FrameStartTriggerMode trigger_mode_; /// @todo Make this property of Camera
	bool running_;
	unsigned long max_data_rate_;
	tPvUint32 sensor_width_, sensor_height_; // full resolution dimensions (maybe should be in lib)
	bool auto_adjust_stream_bytes_per_second_;

	// Hardware triggering
	std::string trig_timestamp_topic_;
	ros::Time trig_time_;

	// ROS messages
	sensor_msgs::Image img_;
	sensor_msgs::CameraInfo cam_info_;

	// Dynamic reconfigure
	typedef prosilica_camera::ProsilicaCameraConfig Config;
	typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
	ReconfigureServer reconfigure_server_;

	// Diagnostics
	ros::Timer diagnostic_timer_;
	boost::shared_ptr<self_test::TestRunner> self_test_;
	diagnostic_updater::Updater diagnostic_;
	std::string hw_id_;
	int count_;
	double desired_freq_;
	static const int WINDOW_SIZE = 5; // remember previous 5s
	unsigned long frames_dropped_total_, frames_completed_total_;
	RollingSum<unsigned long> frames_dropped_acc_, frames_completed_acc_;
	unsigned long packets_missed_total_, packets_received_total_;
	RollingSum<unsigned long> packets_missed_acc_, packets_received_acc_;

	// So we don't get burned by auto-exposure
	unsigned long last_exposure_value_;
	int consecutive_stable_exposures_;

public:
	ProsilicaDriver(const ros::NodeHandle& node_handle);
	~ProsilicaDriver();
	void configure(Config& config, uint32_t level);
	void syncInCallback (const std_msgs::HeaderConstPtr& msg);
	void start();
	void stop();
	void pollCallback(polled_camera::GetPolledImage::Request& req,
	                    polled_camera::GetPolledImage::Response& rsp,
	                    sensor_msgs::Image& image, sensor_msgs::CameraInfo& info);
	static bool frameToImage(tPvFrame* frame, sensor_msgs::Image &image);
	bool processFrame(tPvFrame* frame, sensor_msgs::Image &img, sensor_msgs::CameraInfo &cam_info);
	void publishImage(tPvFrame* frame);
	void loadIntrinsics();
	bool setCameraInfo(sensor_msgs::SetCameraInfo::Request& req,
			sensor_msgs::SetCameraInfo::Response& rsp);
	void normalizeCallback(tPvFrame* frame);
	void normalizeExposure();
	void runDiagnostics();
	void freqStatus(diagnostic_updater::DiagnosticStatusWrapper& status);
	void frameStatistics(diagnostic_updater::DiagnosticStatusWrapper& status);
	void packetStatistics(diagnostic_updater::DiagnosticStatusWrapper& status);
	void packetErrorStatus(diagnostic_updater::DiagnosticStatusWrapper& status);

	////////////////
	// Self tests //
	////////////////
	void infoTest(diagnostic_updater::DiagnosticStatusWrapper& status);
	void attributeTest(diagnostic_updater::DiagnosticStatusWrapper& status);
	void imageTest(diagnostic_updater::DiagnosticStatusWrapper& status);
};

};
