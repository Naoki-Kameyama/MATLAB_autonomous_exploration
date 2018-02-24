#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int32.h>


int i = 0;
int j = 0;


int main(int argc, char** argv){

	ros::init(argc, argv, "odometry_on_map");
	ros::NodeHandle om;

	ros::Publisher oom_pub;
	ros::Publisher count_pub;
	oom_pub = om.advertise<geometry_msgs::Point>("/odom_on_map", 100);
  count_pub = om.advertise<std_msgs::Int32>("/count", 1);


	tf::StampedTransform transform;
  tf::StampedTransform transform2;
	tf::TransformListener listener;
	geometry_msgs::Point odom_m;
	geometry_msgs::Point codom_m1;
	geometry_msgs::Point codom_m2;
	geometry_msgs::Point diff;
	std_msgs::Int32 c;
	c.data = 0;

//	double diff_x[2];
//	double dx;
//	diff_x[j]= 0 ;


	ros::Time now = ros::Time::now();


	while(ros::ok()){

		listener.waitForTransform("/map", "/base_link", now, ros::Duration(1.0));
		listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
		listener.lookupTransform("/map", "/odom",ros::Time(0), transform2);



		odom_m.x = transform.getOrigin().x();
		odom_m.y = transform.getOrigin().y();
		odom_m.z = tf::getYaw(transform.getRotation());

		codom_m1.x = transform2.getOrigin().x();
		codom_m1.y = transform2.getOrigin().y();
		codom_m1.z = tf::getYaw(transform2.getRotation());
//		diff_x[1] = odom_m.x;
//		dx = diff_x[1] - diff_x[0];
//		std::cout << "dx(" << dx << ")" << std::endl;



		if (i ==1){
				diff.x = codom_m2.x - codom_m1.x;
				diff.y = codom_m2.y - codom_m1.y;
 	  		diff.z = codom_m2.z - codom_m1.z;


				std::cout << "diff(" << diff.x << "," << diff.y << "," << diff.z << ")" << std::endl;
				if (diff.x + diff.y + diff.z != 0){
					c.data = c.data + 1;
					std::cout << "count(" << c << ")" << std::endl;
				};

		};

		codom_m2.x = transform2.getOrigin().x();
	  codom_m2.y = transform2.getOrigin().y();
		codom_m2.z = tf::getYaw(transform2.getRotation());

		//std::cout << "odom_m(" << odom_m.x << "," << odom_m.y << "," << odom_m.z << ")" << std::endl;
		//std::cout << "codom_m(" << codom_m1.x << "," << codom_m1.y << "," << codom_m1.z << ")" << std::endl;
		//std::cout << "codom_m2(" << codom_m2.x << "," << codom_m2.y << "," << codom_m2.z << ")" << std::endl;

		oom_pub.publish(odom_m);
		count_pub.publish(c);
		i=1;
		//diff_x[0] = odom_m.x;
	}

	return 0;
}
