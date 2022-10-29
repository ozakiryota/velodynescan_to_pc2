#include <filesystem>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl_conversions/pcl_conversions.h>

#include <velodyne_pointcloud/rawdata.h>
#include <velodyne_pointcloud/point_types.h>

class VelodyneScansToPc2{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
        /*publisher*/
        ros::Publisher pc_pub_;
        /*buffer*/
		/*parameter*/
		std::string load_rosbag_path_;
		std::string save_rosbag_path_;
		std::string save_topic_name_;
		std::string save_topic_frame_;
		std::string calib_path_;
        float min_range_;
        float max_range_;
        float heading_rad_;
        float horizontal_fov_rad_;
        float debug_hz_;
        /*function*/
        void setupPC(sensor_msgs::PointCloud2& pc2, const rosbag::View::iterator& view_itr);
        void publishDebugPC(const sensor_msgs::PointCloud2& pc2);
        void writePC(const sensor_msgs::PointCloud2& pc2);

	public:
		VelodyneScansToPc2();
        void convert();
};

VelodyneScansToPc2::VelodyneScansToPc2()
	: nh_private_("~")
{
	std::cout << "----- velodynescan_to_pc2 -----" << std::endl;

	/*parameter*/
    if(!nh_private_.getParam("load_rosbag_path", load_rosbag_path_)){
        std::cerr << "Set load_rosbag_path." << std::endl; 
        exit(true);
    }
	std::cout << "load_rosbag_path_ = " << load_rosbag_path_ << std::endl;
    nh_private_.param("save_rosbag_path", save_rosbag_path_, std::string(load_rosbag_path_.substr(0, load_rosbag_path_.length() - 4) + "_converted.bag"));
	std::cout << "save_rosbag_path_ = " << save_rosbag_path_ << std::endl;
    nh_private_.param("save_topic_name", save_topic_name_, std::string("/velodyne_points"));
	std::cout << "save_topic_name_ = " << save_topic_name_ << std::endl;
    nh_private_.param("save_topic_frame", save_topic_frame_, std::string("velodyne"));
	std::cout << "save_topic_frame_ = " << save_topic_frame_ << std::endl;
    if(!nh_private_.getParam("calib_path", calib_path_)){
        std::cerr << "Set calib_path." << std::endl; 
        exit(true);
    }
	std::cout << "calib_path_ = " << calib_path_ << std::endl;
    nh_private_.param("min_range", min_range_, float(0));
	std::cout << "min_range_ = " << min_range_ << std::endl;
    nh_private_.param("max_range", max_range_, float(100));
	std::cout << "max_range_ = " << max_range_ << std::endl;
    nh_private_.param("heading_rad", heading_rad_, float(0));
	std::cout << "heading_rad_ = " << heading_rad_ << std::endl;
    nh_private_.param("horizontal_fov_rad", horizontal_fov_rad_, float(2 * M_PI));
	std::cout << "horizontal_fov_rad_ = " << horizontal_fov_rad_ << std::endl;
    nh_private_.param("debug_hz", debug_hz_, float(-1));
	std::cout << "debug_hz_ = " << debug_hz_ << std::endl;

    /*publisher*/
    pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(save_topic_name_, 1);

    /*file*/
    std::filesystem::copy(load_rosbag_path_, save_rosbag_path_, std::filesystem::copy_options::overwrite_existing);
}

void VelodyneScansToPc2::convert()
{
    rosbag::Bag bag;
    try{
        bag.open(load_rosbag_path_, rosbag::bagmode::Read);
    }
    catch(rosbag::BagException const&){
        std::cerr << "Cannot open " << load_rosbag_path_ << std::endl;
        exit(true);
    }

    rosbag::View view;
    rosbag::View::iterator view_itr;
    view.addQuery(bag, rosbag::TypeQuery("velodyne_msgs/VelodyneScan"));
    view_itr = view.begin();

    velodyne_rawdata::RawData v_rawdata;
    v_rawdata.setParameters(min_range_, max_range_, heading_rad_, horizontal_fov_rad_);
    v_rawdata.setupOffline(calib_path_, max_range_, min_range_);

    ros::Rate loop_rate(debug_hz_);
    while(view_itr != view.end()){
        velodyne_msgs::VelodyneScanConstPtr v_scan_ptr = view_itr->instantiate<velodyne_msgs::VelodyneScan>();
        pcl::PointCloud<velodyne_pointcloud::PointXYZIR> v_pc;
        for(velodyne_msgs::VelodynePacket v_packet : v_scan_ptr->packets){
            pcl::PointCloud<velodyne_pointcloud::PointXYZIR> v_tmp_pc;
            v_rawdata.unpack(v_packet, v_tmp_pc);
            v_pc += v_tmp_pc;
        }
        sensor_msgs::PointCloud2 ros_pc;
        pcl::toROSMsg(v_pc, ros_pc);
        setupPC(ros_pc, view_itr);
        writePC(ros_pc);

        publishDebugPC(ros_pc);
        if(debug_hz_ > 0)    loop_rate.sleep();

        view_itr++;
    }

    bag.close();
}

void VelodyneScansToPc2::setupPC(sensor_msgs::PointCloud2& pc2, const rosbag::View::iterator& view_itr)
{
    pc2.header.stamp = view_itr->getTime();
    pc2.header.frame_id = save_topic_frame_;
}

void VelodyneScansToPc2::writePC(const sensor_msgs::PointCloud2& pc2)
{
    rosbag::Bag bag;
    try{
        bag.open(save_rosbag_path_, rosbag::bagmode::Append);
    }
    catch(rosbag::BagException const&){
        std::cerr << "Cannot open " << save_rosbag_path_ << std::endl;
        exit(true);
    }
    bag.write(save_topic_name_, pc2.header.stamp, pc2);
    bag.close();
}

void VelodyneScansToPc2::publishDebugPC(const sensor_msgs::PointCloud2& pc2)
{
    pc_pub_.publish(pc2);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "velodynescan_to_pc2");
	
	VelodyneScansToPc2 velodynescan_to_pc2;
    velodynescan_to_pc2.convert();
}