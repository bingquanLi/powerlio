#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/conversions.h>
#include <pcl/filters/passthrough.h>  
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/crop_box.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include <pcl/filters/box_clipper3D.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>

using namespace std;     

ros::Publisher front_cam_pub;
ros::Publisher back_cam_pub;
ros::Publisher mid360_pub;
ros::Publisher marker_pub;

geometry_msgs::TransformStamped LF_FOOT_transform;
geometry_msgs::TransformStamped LF_calf_transform;
geometry_msgs::TransformStamped LF_thigh_transform;
geometry_msgs::TransformStamped RF_FOOT_transform;
geometry_msgs::TransformStamped RF_calf_transform;
geometry_msgs::TransformStamped RF_thigh_transform;

geometry_msgs::TransformStamped front_cam_to_LF_FOOT;

std::string front_cam_input,front_cam_output,back_cam_input,back_cam_output;
bool use_front_cam=false,use_back_cam=false;

pcl::PointCloud<pcl::PointXYZ>::Ptr BOX(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,double x,double y,double z,double d)
{
  pcl::CropBox<pcl::PointXYZ> crop;
  pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
  
  double min_x = x-d;
  double max_x = x+d;
  double min_y = y-d;
  double max_y = y+d;
  double min_z = z-d;
  double max_z = z+d;

  crop.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1.0));
  crop.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1.0));

  crop.setInputCloud(cloud);
  crop.setNegative(true); // true表示删除索引指向的点
  crop.filter(*result);
  //std::cout << "Point Cloud: " << result->size() << " points" << std::endl;
  
  return result;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Reserve_specific_areas(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,float x1,float x2,float y1,float y2,float z1,float z2)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_z(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_z_end(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_x(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_x_end(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_y(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_y_end(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PassThrough<pcl::PointXYZI> pass; //创建直通滤波器对象
    pass.setInputCloud(cloud); //设置输入的点云
    pass.setFilterFieldName("z"); //设置过滤时所需要点云类型为Z字
    pass.setFilterLimits(z1, z2); //设置在过滤字段的范围
    pass.setFilterLimitsNegative(true); //设置保留还是过滤掉字段范围内的点，设置为true表示过滤掉字段范围内的点
    pass.filter(*cloud_z); //执行滤波
    pass.setFilterLimitsNegative(false); //设置保留还是过滤掉字段范围内的点，设置为true表示过滤掉字段范围内的点
    pass.filter(*cloud_z_end); //执行滤波

    pass.setInputCloud(cloud_z_end); //设置输入的点云
    pass.setFilterFieldName("x"); //设置过滤时所需要点云类型为Z字段
    pass.setFilterLimits(x1, x2); //设置在过滤字段的范围
    pass.setFilterLimitsNegative(true); //设置保留还是过滤掉字段范围内的点，设置为true表示过滤掉字段范围内的点
    pass.filter(*cloud_x); //执行滤波
    pass.setFilterLimitsNegative(false); //设置保留还是过滤掉字段范围内的点，设置为true表示过滤掉字段范围内的点
    pass.filter(*cloud_x_end); //执行滤波

    pass.setInputCloud(cloud_x_end); //设置输入的点云
    pass.setFilterFieldName("y"); //设置过滤时所需要点云类型为Z字段
    pass.setFilterLimits(y1, y2); //设置在过滤字段的范围
    pass.setFilterLimitsNegative(true); //设置保留还是过滤掉字段范围内的点，设置为true表示过滤掉字段范围内的点
    pass.filter(*cloud_y); //执行滤波
    pass.setFilterLimitsNegative(false); //设置保留还是过滤掉字段范围内的点，设置为true表示过滤掉字段范围内的点
    pass.filter(*cloud_y_end); //执行滤波
    *cloud_z+=*cloud_y;
    *cloud_z+=*cloud_x;
    return cloud_z;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr save_specific_areas(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,float x1,float x2,float y1,float y2,float z1,float z2)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_end(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass; //创建直通滤波器对象

    pcl::RandomSample<pcl::PointXYZ> rs;
    rs.setInputCloud (cloud);
    rs.setSample (3000); // 设置要采样的点的数量
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled (new pcl::PointCloud<pcl::PointXYZ>);
    rs.filter (*cloud);

    pass.setInputCloud(cloud); //设置输入的点云
    pass.setFilterFieldName("z"); //设置过滤时所需要点云类型为Z字
    pass.setFilterLimits(z1, z2); //设置在过滤字段的范围
    pass.setFilterLimitsNegative(false); //设置保留还是过滤掉字段范围内的点，设置为true表示过滤掉字段范围内的点
    pass.filter(*cloud_end); //执行滤波   
    pass.setInputCloud(cloud_end); //设置输入的点云 
    pass.setFilterFieldName("y"); //设置过滤时所需要点云类型为Z字
    pass.setFilterLimits(y1, y2); //设置在过滤字段的范围
    pass.setFilterLimitsNegative(false); //设置保留还是过滤掉字段范围内的点，设置为true表示过滤掉字段范围内的点  
    pass.filter(*cloud_end); //执行滤波
    pass.setInputCloud(cloud_end); //设置输入的点云  
    pass.setFilterFieldName("x"); //设置过滤时所需要点云类型为Z字
    pass.setFilterLimits(x1, x2); //设置在过滤字段的范围
    pass.setFilterLimitsNegative(false); //设置保留还是过滤掉字段范围内的点，设置为true表示过滤掉字段范围内的点
    pass.filter(*cloud_end); //执行滤波 
    
    // pcl::VoxelGrid<pcl::PointXYZ> vg;		//创建滤波器对象
	  // vg.setInputCloud(cloud_end);				//设置待滤波点云
	  // vg.setLeafSize(0.01f, 0.01f, 0.01f);	//设置体素大小
	  // vg.filter(*cloud_end);	 //执行滤波，保存滤波结果于cloud_filtered

    return cloud_end;
}

void back_cloud_cb (const sensor_msgs::PointCloud2& cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
  //将ros点云转换为pcl点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(cloud_msg, *cloud);

  //处理操作
  //result=Reserve_specific_areas(cloud,-0.65,0.65,-0.4,0.4,-0.1,0.5);
  result=save_specific_areas(cloud,-1,1.5,-1,1,-1,1);

  //将处理后的pcl点云转换为ros点云并发布
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*result,output);
  output.header = cloud_msg.header;
  back_cam_pub.publish (output);
}

void front_cloud_cb (const sensor_msgs::PointCloud2& cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(cloud_msg, *result);

  // double mid_x_L=(LF_FOOT_transform.transform.translation.x+LF_calf_transform.transform.translation.x)/2;
  // double mid_y_L=(LF_FOOT_transform.transform.translation.y+LF_calf_transform.transform.translation.y)/2;
  // double mid_z_L=(LF_FOOT_transform.transform.translation.z+LF_calf_transform.transform.translation.z)/2;
  //处理操作
  //result=Reserve_specific_areas(cloud,-0.65,0.65,-0.4,0.4,-0.1,0.5);
  result=save_specific_areas(result,-1,1,-1,1,-1,1);
  // result=BOX(result,front_cam_to_LF_FOOT.transform.translation.x,
  //                   front_cam_to_LF_FOOT.transform.translation.y,
  //                   front_cam_to_LF_FOOT.transform.translation.z,
  //                   0.065);  
  //将处理后的pcl点云转换为ros点云并发布
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*result,output);
  output.header = cloud_msg.header;
  front_cam_pub.publish (output);
}

void mid360_cloud_cb (const sensor_msgs::PointCloud2& cloud_msg)
{
  pcl::CropBox<pcl::PointXYZ> crop;

  pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(cloud_msg, *result);
  // cout << "start:"<<  result->size()<< endl;
  //处理操作
  double mid_x_L=(LF_FOOT_transform.transform.translation.x+LF_calf_transform.transform.translation.x)/2;
  double mid_y_L=(LF_FOOT_transform.transform.translation.y+LF_calf_transform.transform.translation.y)/2;
  double mid_z_L=(LF_FOOT_transform.transform.translation.z+LF_calf_transform.transform.translation.z)/2;
  double mid_x_th_L=(LF_thigh_transform.transform.translation.x+LF_calf_transform.transform.translation.x)/2;
  double mid_y_th_L=(LF_thigh_transform.transform.translation.y+LF_calf_transform.transform.translation.y)/2;
  double mid_z_th_L=(LF_thigh_transform.transform.translation.z+LF_calf_transform.transform.translation.z)/2;
  double mid_x_down_L=(LF_FOOT_transform.transform.translation.x+mid_x_L)/2;
  double mid_y_down_L=(LF_FOOT_transform.transform.translation.y+mid_y_L)/2;
  double mid_z_down_L=(LF_FOOT_transform.transform.translation.z+mid_z_L)/2;
  double mid_x_up_L=(LF_calf_transform.transform.translation.x+mid_x_L)/2;
  double mid_y_up_L=(LF_calf_transform.transform.translation.y+mid_y_L)/2;
  double mid_z_up_L=(LF_calf_transform.transform.translation.z+mid_z_L)/2;
  double mid_x_th_down_L=(LF_calf_transform.transform.translation.x+mid_x_th_L)/2;
  double mid_y_th_down_L=(LF_calf_transform.transform.translation.y+mid_y_th_L)/2;
  double mid_z_th_down_L=(LF_calf_transform.transform.translation.z+mid_z_th_L)/2;

  double mid_x_R=(RF_FOOT_transform.transform.translation.x+RF_calf_transform.transform.translation.x)/2;
  double mid_y_R=(RF_FOOT_transform.transform.translation.y+RF_calf_transform.transform.translation.y)/2;
  double mid_z_R=(RF_FOOT_transform.transform.translation.z+RF_calf_transform.transform.translation.z)/2;
  double mid_x_th_R=(RF_thigh_transform.transform.translation.x+RF_calf_transform.transform.translation.x)/2;
  double mid_y_th_R=(RF_thigh_transform.transform.translation.y+RF_calf_transform.transform.translation.y)/2;
  double mid_z_th_R=(RF_thigh_transform.transform.translation.z+RF_calf_transform.transform.translation.z)/2;
  double mid_x_down_R=(RF_FOOT_transform.transform.translation.x+mid_x_R)/2;
  double mid_y_down_R=(RF_FOOT_transform.transform.translation.y+mid_y_R)/2;
  double mid_z_down_R=(RF_FOOT_transform.transform.translation.z+mid_z_R)/2;
  double mid_x_up_R=(RF_calf_transform.transform.translation.x+mid_x_R)/2;
  double mid_y_up_R=(RF_calf_transform.transform.translation.y+mid_y_R)/2;
  double mid_z_up_R=(RF_calf_transform.transform.translation.z+mid_z_R)/2;
  double mid_x_th_down_R=(RF_calf_transform.transform.translation.x+mid_x_th_R)/2;
  double mid_y_th_down_R=(RF_calf_transform.transform.translation.y+mid_y_th_R)/2;
  double mid_z_th_down_R=(RF_calf_transform.transform.translation.z+mid_z_th_R)/2;

  result=BOX(result,LF_FOOT_transform.transform.translation.x,
                    LF_FOOT_transform.transform.translation.y,
                    LF_FOOT_transform.transform.translation.z,
                    0.065);      
  result=BOX(result,LF_calf_transform.transform.translation.x,
                    LF_calf_transform.transform.translation.y,
                    LF_calf_transform.transform.translation.z,
                    0.06);
  result=BOX(result,mid_x_L,
                    mid_y_L,
                    mid_z_L,
                    0.06);
  result=BOX(result,mid_x_th_L,
                    mid_y_th_L,
                    mid_z_th_L,
                    0.06);
  result=BOX(result,mid_x_down_L,
                    mid_y_down_L,
                    mid_z_down_L,
                    0.065);
  result=BOX(result,mid_x_up_L,
                    mid_y_up_L,
                    mid_z_up_L,
                    0.06);
  result=BOX(result,mid_x_th_down_L,
                    mid_y_th_down_L,
                    mid_z_th_down_L,
                    0.06);                    

  result=BOX(result,RF_FOOT_transform.transform.translation.x,
                    RF_FOOT_transform.transform.translation.y,
                    RF_FOOT_transform.transform.translation.z,
                    0.065);
  result=BOX(result,RF_calf_transform.transform.translation.x,
                    RF_calf_transform.transform.translation.y,
                    RF_calf_transform.transform.translation.z,
                    0.06);
  result=BOX(result,mid_x_R,
                    mid_y_R,
                    mid_z_R,
                    0.06);
  result=BOX(result,mid_x_th_R,
                    mid_y_th_R,
                    mid_z_th_R,
                    0.06);
  result=BOX(result,mid_x_down_R,
                    mid_y_down_R,
                    mid_z_down_R,
                    0.065);
  result=BOX(result,mid_x_up_R,
                    mid_y_up_R,
                    mid_z_up_R,
                    0.06);
  result=BOX(result,mid_x_th_down_R,
                    mid_y_th_down_R,
                    mid_z_th_down_R,
                    0.06);  

//   // 创建半径滤波（模板）类对象
//   pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;
//   // 设置输入点云
//   ror.setInputCloud(result);
//   // 设置搜索半径
//   ror.setRadiusSearch(0.035f);
//   // 设置半径范围内的最少点数阈值
//   ror.setMinNeighborsInRadius(1);
//   // 执行滤波，并带出结果数据
//   ror.filter(*result);  //保存滤波结果到cloud_filtered
        
  //将处理后的pcl点云转换为ros点云并发布
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*result,output);
  mid360_pub.publish (output);
}

void show_marker (const geometry_msgs::TransformStamped& transform,const ros::Publisher& marker_pub)
{
    // 设置 Marker 的基本属性
    visualization_msgs::Marker marker;
    marker.header.frame_id = "lidar";   // 参考坐标系
    marker.header.stamp = ros::Time::now(); // 时间戳
    marker.ns = "box_LF_FOOT";            // 命名空间
    marker.id = 0;                          // 唯一 ID
    marker.type = visualization_msgs::Marker::CUBE; // Marker 类型：立方体
    marker.action = visualization_msgs::Marker::ADD; // 添加 Marker
    // 设置方框的位置和朝向
    marker.pose.position.x = transform.transform.translation.x;
    marker.pose.position.y = transform.transform.translation.y;
    marker.pose.position.z = transform.transform.translation.z;
    marker.pose.orientation.x = transform.transform.rotation.x;
    marker.pose.orientation.y = transform.transform.rotation.y;
    marker.pose.orientation.z = transform.transform.rotation.z;
    marker.pose.orientation.w = transform.transform.rotation.w;
    // 设置方框的大小（长、宽、高）
    marker.scale.x = 0.08;
    marker.scale.y = 0.08;
    marker.scale.z = 0.08;
    // marker.scale.x = 0.5;
    // marker.scale.y = 0.5;
    // marker.scale.z = 0.5;
    // 设置方框的颜色和透明度（RGBA）
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.5f; // 完全不透明
    // 设置方框的生命周期（0 表示永久存在）
    marker.lifetime = ros::Duration(0.01);
    marker_pub.publish(marker);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_write");
  ros::NodeHandle nh;

  nh.getParam("front_cam_input", front_cam_input); 
  nh.getParam("front_cam_output", front_cam_output); 

  nh.getParam("back_cam_input", back_cam_input); 
  nh.getParam("back_cam_output", back_cam_output); 

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber back_cam_sub = nh.subscribe (back_cam_input, 1, back_cloud_cb);
    // Create a ROS publisher for the output point cloud
    back_cam_pub = nh.advertise<sensor_msgs::PointCloud2> (back_cam_output, 1);

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber front_cam_sub = nh.subscribe (front_cam_input, 1, front_cloud_cb);
    // Create a ROS publisher for the output point cloud
    front_cam_pub = nh.advertise<sensor_msgs::PointCloud2> (front_cam_output, 1);


  // Spin
  ros::spin ();
}

// int main(int argc, char** argv) {

//     ros::init(argc, argv, "pcl_write");
//     ros::NodeHandle node;

//     tf2_ros::Buffer tfBuffer;
//     tf2_ros::TransformListener tfListener(tfBuffer);

//     node.getParam("front_cam_input", front_cam_input); 
//     node.getParam("front_cam_output", front_cam_output); 

//     node.getParam("back_cam_input", back_cam_input); 
//     node.getParam("back_cam_output", back_cam_output); 

//     // Create a ROS subscriber for the input point cloud
//     ros::Subscriber back_cam_sub = node.subscribe (back_cam_input, 100, back_cloud_cb);
//     // Create a ROS publisher for the output point cloud
//     back_cam_pub = node.advertise<sensor_msgs::PointCloud2> (back_cam_output, 100);

//     // Create a ROS subscriber for the input point cloud
//     // ros::Subscriber front_cam_sub = node.subscribe (front_cam_input, 100, front_cloud_cb);
//     // Create a ROS publisher for the output point cloud
//     // front_cam_pub = node.advertise<sensor_msgs::PointCloud2> (front_cam_output, 100);

//     // Create a ROS subscriber for the input point cloud
//     ros::Subscriber mid360_sub = node.subscribe ("/livox/points", 1000, mid360_cloud_cb);
//     // Create a ROS publisher for the output point cloud
//     mid360_pub = node.advertise<sensor_msgs::PointCloud2> ("/mid360_for_map", 1000);

//     ros::Rate rate(100.0);
//     while (node.ok()) {
//         try {
//             LF_FOOT_transform = tfBuffer.lookupTransform("lidar", "LF_FOOT", ros::Time(0));
//         } catch (tf2::TransformException &ex) {
//             ROS_WARN("Transform lookup failed: %s", ex.what());
//             ros::Duration(1.0).sleep();
//             continue;
//         }
//         try {
//             LF_calf_transform = tfBuffer.lookupTransform("lidar", "LF_calf", ros::Time(0));
//         } catch (tf2::TransformException &ex) {
//             ROS_WARN("Transform lookup failed: %s", ex.what());
//             ros::Duration(1.0).sleep();
//             continue;
//         }
//         try {
//             LF_thigh_transform = tfBuffer.lookupTransform("lidar", "LF_thigh", ros::Time(0));
//         } catch (tf2::TransformException &ex) {
//             ROS_WARN("Transform lookup failed: %s", ex.what());
//             ros::Duration(1.0).sleep();
//             continue;
//         }


//         try {
//             RF_FOOT_transform = tfBuffer.lookupTransform("lidar", "RF_FOOT", ros::Time(0));
//         } catch (tf2::TransformException &ex) {
//             ROS_WARN("Transform lookup failed: %s", ex.what());
//             ros::Duration(1.0).sleep();
//             continue;
//         }
//         try {
//             RF_calf_transform = tfBuffer.lookupTransform("lidar", "RF_calf", ros::Time(0));
//         } catch (tf2::TransformException &ex) {
//             ROS_WARN("Transform lookup failed: %s", ex.what());
//             ros::Duration(1.0).sleep();
//             continue;
//         }
//         try {
//             RF_thigh_transform = tfBuffer.lookupTransform("lidar", "RF_thigh", ros::Time(0));
//         } catch (tf2::TransformException &ex) {
//             ROS_WARN("Transform lookup failed: %s", ex.what());
//             ros::Duration(1.0).sleep();
//             continue;
//         }
//         ros::spinOnce();
//         rate.sleep();
//     }

//     return 0;
// }