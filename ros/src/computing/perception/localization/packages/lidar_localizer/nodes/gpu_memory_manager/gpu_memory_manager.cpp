#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <memory_manager/manager.h>

static ros::Publisher gpu_pub;
static ros::Publisher data_pub;
static ros::Publisher ret_pub;

static GpuIpcTest ins;

autoware_msgs::gpu_handle handle_msg;
std_msgs::Int16 data_msg;
std_msgs::Bool ret_msg;

unsigned char *buf;

static int count = 0;

static int node_count = 0;
static int free_count = 0;
static int node_que = 0;

void setHandleMsg() 
{
    handle_msg.data.clear();
    for (int i = 0; i < 64; i++) {
        handle_msg.data.push_back(buf[i]);
    }

    free(buf);

    return;
}

void publishData() 
{
    node_count += node_que;
    node_que = 0;

    if (node_count != 0) {
        data_msg.data = ins.updateData();
        data_pub.publish(data_msg);

        free_count = 0;
    }

    return;
}

void lidar_callback(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    ret_pub.publish(ret_msg);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    pcl::PointXYZ *input = cloud.points.data();

    int points_num = cloud.size();

    float *x, *y, *z;
    x = (float*)malloc(sizeof(float)*points_num);
    y = (float*)malloc(sizeof(float)*points_num);
    z = (float*)malloc(sizeof(float)*points_num);

    for (int i = 0; i < points_num; i++) {
        x[i] = input[i].x;
        y[i] = input[i].y;
        z[i] = input[i].z;
    }
    
    ins.storeBuffer(x, y, z, cloud.size());

    if (node_count <= free_count) {
        publishData();
    }

    return;
}

void free_callback(std_msgs::Bool msg) {
    free_count++;

    if ((node_count <= free_count) && (ins.notEmpty())) {
        publishData();
    }

    return;
}

void ack_callback(std_msgs::Bool msg)
{
    ROS_INFO("ack_callback");
    sleep(1);
    if (msg.data) ROS_INFO("receive ack");
    else ROS_INFO("receive ready");
    if (msg.data) gpu_pub.publish(handle_msg);
    else node_que++;

    return;
}

int main (int argc, char *argv[])
{
    ros::init(argc, argv, "gpu_memory_manager");

    ros::NodeHandle n;

    gpu_pub = n.advertise<autoware_msgs::gpu_handle>("gpu_handler", 10);
    data_pub = n.advertise<std_msgs::Int16>("data_size", 10);
    ret_pub = n.advertise<std_msgs::Bool>("ret", 10);
    ret_msg.data = true;

    buf = ins.initGpuMemory();

    setHandleMsg();

    sleep(1);

    gpu_pub.publish(handle_msg);
    ROS_INFO("publish gpu_handle");

    ros::Subscriber ack_sub = n.subscribe("node_ack", 10, ack_callback);
    ros::Subscriber lidar_sub = n.subscribe("points_raw", 10, lidar_callback);
    ros::Subscriber free_sub = n.subscribe("free_signal", 10, free_callback);

    ros::spin();

    ins.freeResources();

    return 0;
}
