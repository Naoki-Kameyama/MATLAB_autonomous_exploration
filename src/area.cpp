#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <fstream>//ファイル出力用
#include <sys/stat.h>//ディレクトリ作成用

using namespace std;
ofstream ofs("./area_data/area.csv"); //ファイル出力ストリーム

ros::Subscriber map_sub;
int i;
ros::Time start;
ros::Duration end;
double end_sec;


void area(const nav_msgs::OccupancyGrid::ConstPtr& msg){

	nav_msgs::MapMetaData info = msg->info;//地図の設定を取得
	std::vector<int8_t> data = msg->data;//地図の値を取得
  int x = info.width;//地図の横サイズ
  int y = info.height;//地図の縦サイズ
  int m=0;
	double menseki;
	//std::cout << "area" << std::endl;
  //std::cout << x*y << std::endl;
	for(i=0; i<x*y; i++){
      //std::cout <<  +data[i] << std::endl;
      if(data[i]!=-1){
        m+=1;
      }
  	}
		menseki = m*0.0025;//calcurate area
		end = ros::Time::now()-start;
		end_sec = end.toSec();
		std::cout <<"Time: " << end_sec <<"   area: "<< menseki << std::endl;
		ofs << end_sec << " " << menseki << "\n";
}


int main(int argc, char** argv){
	mkdir("area_data", 0755);
	ros::init(argc, argv, "area");
	ros::NodeHandle nh ;
  start = ros::Time::now();

  map_sub = nh.subscribe("/map",1,area);
	ros::spin();
	return 0;
}
