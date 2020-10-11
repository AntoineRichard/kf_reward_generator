#ifndef LZR2CMAP_H
#define LZR2CMAP_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float32MultiArray.h>
#include <lzr2cmap/LaserCostMap.h>//custom message
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

class LZR2CMap {
    /*
    Generates a cost map when a laser scan is received
    The message published is a custom LaserCostMap, where the data is stored in a Float32MultiArray
    data[0] contains the information
        data[0].layout information on the multiarray
        data[0].data the actual data
    more information on multiarray: http://docs.ros.org/melodic/api/std_msgs/html/msg/MultiArrayLayout.html
    */
    
	private:
		ros::NodeHandle nh_;
		ros::Subscriber lzr_sub_;
		ros::Publisher cmap_pub_;
		ros::Publisher floatmap_pub_;
        
        // settings
		float local_grid_res_;
		float local_grid_front_size_;
		float local_grid_rear_size_;
		float local_grid_side_size_;
		float dist_to_shore_;
		float safe_to_shore_;
		float max_gap_;
		std::string lzr_frame_;
        bool publish_occupancy_grid_;
		int size_;
		int kernel_size_;
		int safe_kernel_size_;
		int hskernel_;
		int hssafety_;
        int safety_cost_;
        float blur_kernel_size_;
        cv::Size blur_ksize_;
        
        // ROS msgs
		nav_msgs::OccupancyGrid occupancy_grid_;
        laser2costmap::LaserCostMap floatmap_;

        //vectors used to draw the map from the laser points
        typedef std::vector< std::array<int,2> > PointsVector; 
        PointsVector buffer_;
        PointsVector inner_shores_;
        PointsVector points_;

        // Map
		cv::Mat_<uint8_t> map_;
		cv::Mat_<uint8_t> zero_map_;
		cv::Mat_<uint8_t> one_map_;
		cv::Mat_<uint8_t> kernel_;
		//cv::Mat_<float> safety_;
		cv::Mat_<uint8_t> safety_;
		cv::Mat_<uint8_t> thin_map_;
		cv::Mat_<int> label_map_;
		cv::Mat_<float> cost_map_;
		cv::Mat_<float> cropped_map_;
		cv::Mat_<float> smoothed_map_;
		cv::Mat_<uint8_t> n_cost_map_;

        cv::Range cropw_;
        cv::Range croph_;
        
        // laser -to-> map functions
		void initializeMap();
		void refreshMap();
        void setMapBlockValue(int, int, cv::Mat&, int, cv::Mat&, int);
        void genContourMap();
		void genCostMap();
        
        void addToBuffer(const sensor_msgs::LaserScanConstPtr &);
		void filterPoints(PointsVector&, PointsVector&);
		void interpolatePoints(PointsVector&, PointsVector&);
        double calcDist(const PointsVector&, size_t, size_t);
        

        // Callback
		void LZRCallback(const sensor_msgs::LaserScanConstPtr&);
        void publishOccupancyGrid();
        void publishFloatMap();
	public:
		LZR2CMap();
};

#endif // LZR2CMAP_H
