#include <ros/ros.h>
#include <string>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <dynamic_reconfigure/server.h>
#include <iterator>
#include <string>
#include <vector>

// node specific includes
#include <pointcloud_to_laserscan/slicerConfig.h>


class PointCloudToLaserScan{
    public: 
        PointCloudToLaserScan(ros::NodeHandle);


    private:
        float z_min_ = -0.1;
        float z_max_ = 0.3;
        sensor_msgs::PointCloud2 pointcloud_;
        std::string point_cloud_topic_;
        float voxel_leaf_size_ = 0.01;
        ros::NodeHandle n_;
        pcl::PCLPointCloud2::Ptr cloud_ptr_;

        std::string target_frame_ = "velodyne";
        double tolerance_;
        double min_height_;
        double max_height_;
        double angle_min_;
        double angle_max_;
        double angle_increment_;
        double scan_time_;
        double range_min_;
        double range_max_;
        double inf_epsilon_;
        int concurrency_level;
        bool use_inf_;

        dynamic_reconfigure::Server<pointcloud_to_laserscan::slicerConfig>dynamic_server;
        dynamic_reconfigure::Server<pointcloud_to_laserscan::slicerConfig>::CallbackType f_;

        ros::Subscriber pc_sub;
        ros::Publisher scan_pub_;
        void pointcloud_cb(const sensor_msgs::PointCloud2 &);
        void dynamic_cb(pointcloud_to_laserscan::slicerConfig &, uint32_t);
        void setupSub();


};

PointCloudToLaserScan::PointCloudToLaserScan(ros::NodeHandle n){
  this->n_ = n;
  this->n_.param<std::string>("cloud_in", this->point_cloud_topic_, "/cloud_in");
  this->scan_pub_ = this->n_.advertise<sensor_msgs::LaserScan>("/scan", 1);


  n_.param<std::string>("target_frame", this->target_frame_, "");
  n_.param<double>("transform_tolerance", this->tolerance_, 0.01);
  n_.param<double>("min_height", this->min_height_, std::numeric_limits<double>::min());
  n_.param<double>("max_height", this->max_height_, std::numeric_limits<double>::max());
  n_.param<double>("angle_min", this->angle_min_, -M_PI);
  n_.param<double>("angle_max", this->angle_max_, M_PI);
  n_.param<double>("angle_increment", this->angle_increment_, M_PI / 180.0);
  n_.param<double>("scan_time", this->scan_time_, 1.0 / 30.0);
  n_.param<double>("range_min", this->range_min_, 0.0);
  n_.param<double>("range_max", this->range_max_, std::numeric_limits<double>::max());
  n_.param<double>("inf_epsilon", this->inf_epsilon_, 1.0);
  n_.param<int>("concurrency_level", this->concurrency_level, 1);
  n_.param<bool>("use_inf", this->use_inf_, true);
  n_.setParam("this",1);

  setupSub();

}

void PointCloudToLaserScan::setupSub() {
  this->pc_sub = this->n_.subscribe(this->point_cloud_topic_, 1,
                                    &PointCloudToLaserScan::pointcloud_cb, this);

  this->f_ = boost::bind(&PointCloudToLaserScan::dynamic_cb, this, _1, _2);
  this->dynamic_server.setCallback(f_);

  usleep(2500000);
  ros::spinOnce();
}

void PointCloudToLaserScan::pointcloud_cb(const sensor_msgs::PointCloud2 &msg){
  //ROS_WARN("GOT CLOUD");
  this->pointcloud_ = msg;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2 temp_cloud;
  pcl::fromROSMsg(this->pointcloud_, *pcl_cloud);
  pcl::toPCLPointCloud2(*pcl_cloud.get(), temp_cloud);
  pcl::PCLPointCloud2Ptr ptr_cloud(new pcl::PCLPointCloud2(temp_cloud));

  this->cloud_ptr_ = ptr_cloud;

  std::cout << " tc "<<(temp_cloud.width*temp_cloud.height) << std::endl;


  pcl::PCLPointCloud2 passthrough_cloud;
  pcl::PassThrough<pcl::PCLPointCloud2> pass;
  pass.setInputCloud(this->cloud_ptr_);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(this->z_min_, this->z_max_);
  // pass.setFilterLimitsNegative (true);
  pass.filter(passthrough_cloud);

std::cout << "z " <<this->z_min_ << " " << this->z_max_ << std::endl;

  std::cout << "pc " <<(passthrough_cloud.width*passthrough_cloud.height) << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB> pass_cloud;
  pcl::fromPCLPointCloud2(passthrough_cloud, pass_cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pass_cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  *pass_cloud_ptr = pass_cloud;    

  // voxel filter to downsample
  pcl::PointCloud<pcl::PointXYZRGB> vox_cloud;
  pcl::VoxelGrid<pcl::PointXYZRGB> vox;
  vox.setInputCloud(pass_cloud_ptr);
  vox.setLeafSize(this->voxel_leaf_size_, this->voxel_leaf_size_,
                  this->voxel_leaf_size_);
  vox.filter(vox_cloud);

  std::cout << "vc " <<(vox_cloud.width*vox_cloud.height) << std::endl;  

  //pcl::fromPCLPointCloud2(cloud_filtered, vox_cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr vox_cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  *vox_cloud_ptr = vox_cloud;

  pcl::PCLPointCloud2 cloud_reduced;
  pcl::toPCLPointCloud2(vox_cloud, cloud_reduced);
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_reduced, output);  

  std::cout << "oc " <<(output.width) << std::endl;  



  //ROS_WARN("CLOUD FILTERED");

  sensor_msgs::LaserScan output_scan;
  output_scan.header = this->pointcloud_.header;

  output_scan.header.frame_id = target_frame_;

  output_scan.angle_min = this->angle_min_;
  output_scan.angle_max = this->angle_max_;
  output_scan.angle_increment = this->angle_increment_;
  output_scan.time_increment = 0.0;
  output_scan.scan_time = this->scan_time_;
  output_scan.range_min = this->range_min_;
  output_scan.range_max = this->range_max_;

  // determine amount of rays to create
  uint32_t ranges_size = std::ceil((output_scan.angle_max - output_scan.angle_min) / output_scan.angle_increment);
  output_scan.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());


  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(output, "x"), iter_y(output, "y"),
       iter_z(output, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
   double angle = - atan2(*iter_x, *iter_z); 


    // overwrite range at laserscan ray if new range is smaller
    int index = (angle - output_scan.angle_min) / output_scan.angle_increment;
    double range = hypot(*iter_x, *iter_z);

    if (range < output_scan.ranges[index])
    {
      output_scan.ranges[index] = range;
    }
  }
  scan_pub_.publish(output_scan);

    //ROS_WARN("CLOUD PUBLISHED");

}

void PointCloudToLaserScan::dynamic_cb(pointcloud_to_laserscan::slicerConfig &config,
                          uint32_t level) {

  ROS_WARN("RECONFIG CALLED");

  if (config.z_min < config.z_max) {
    this->z_min_ = config.z_min;
    this->z_max_ = config.z_max;
  } else {
    this->z_max_ = config.z_min;
    this->z_min_ = config.z_max;
  }
}

int main(int argc, char** argv)
    {
    ros::init(argc, argv, "pointcloud_to_laserscan");
    ros::NodeHandle private_nh("~");
    PointCloudToLaserScan pc2ls(private_nh);

    while (ros::ok()) {
      ros::spinOnce();
      ros::Rate r(10);
      r.sleep();        
    }


    }
