#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TransformStamped.h>

void GoundTruthCallback(const tf2_msgs::TFMessage::ConstPtr& msg,std::ofstream& file)
{
    for(int index_i = 0;index_i< msg->transforms.size();index_i++)
    {
        geometry_msgs::TransformStamped one_tf;
        one_tf = msg->transforms[index_i];
        if(file.is_open())
        {
            file << std::fixed << one_tf.header.stamp.toSec() << " "
                               << one_tf.transform.translation.x    << " "
                               << one_tf.transform.translation.y    << " "
                               << one_tf.transform.translation.z    << " "
                               << one_tf.transform.rotation.x   << " "
                               << one_tf.transform.rotation.y   << " "
                               << one_tf.transform.rotation.z   << " "
                               << one_tf.transform.rotation.w   << "\n";
            std::cout << std::fixed << one_tf.header.stamp.toSec() << " "
                               << one_tf.transform.translation.x    << " "
                               << one_tf.transform.translation.y    << " "
                               << one_tf.transform.translation.z    << " "
                               << one_tf.transform.rotation.x   << " "
                               << one_tf.transform.rotation.y   << " "
                               << one_tf.transform.rotation.z   << " "
                               << one_tf.transform.rotation.w   << std::endl;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gt_write_show");

    ros::NodeHandle n;

    std::ofstream file;

    file.open("/home/yyj/path/gt_path.txt",std::ios::out);

    ros::Subscriber gt_sub = n.subscribe<tf2_msgs::TFMessage>("/gt", 1,boost::bind(&GoundTruthCallback,_1,boost::ref(file)));
    ros::spin();
    file.close();

    return 0;
}