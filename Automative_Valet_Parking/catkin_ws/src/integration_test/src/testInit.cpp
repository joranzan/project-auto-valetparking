#include <unistd.h>
#include <iostream>
#include <sys/wait.h>
#include <errno.h>
#include <cstdio>
#include "ros/ros.h"
#include "std_msgs/Int8.h"
#include <mutex>
#include <cstring>
#include <vector>
#include <thread>
#define MAX_CMD_NUM 6
#define MAX_ARG_NUM 10
#define MAX_PATH_LENGTH 50

using namespace std;

pid_t pidList[MAX_CMD_NUM];

vector<string> cmdList[MAX_CMD_NUM] = {
	{"",""},
	//{"/opt/ros/melodic/bin/rviz","rviz","rviz"},
	// {"/opt/ros/melodic/bin/roslaunch","roslaunch","ssafy_1", "talker_listener_1.launch"},
	{"/opt/ros/melodic/bin/rosrun", "rosrun", "ssafy_3", "load_named_scenario.py", "scenario1.json", "1"},
	{"/opt/ros/melodic/bin/roslaunch","roslaunch","rrt_drive2", "parking_lot_v_rrt_forward.launch"},
	{"/opt/ros/melodic/bin/rosrun", "rosrun", "ssafy_3", "load_named_scenario.py", "scenario3.json", "3"},
	{"/opt/ros/melodic/bin/roslaunch","roslaunch","ssafy_3", "parking_lot_se3.launch"},
	{"/opt/ros/melodic/bin/roslaunch","roslaunch","ssafy_3", "parking_lot_exit.launch"}
};

int currentCmdIdx = 1;
mutex idxMutex;

void endMsgCallback(const std_msgs::Int8::ConstPtr& msg){
	cout<<"in callback\n";
	ROS_INFO("Receive data = %d", msg->data);
	idxMutex.lock();
	if((int) msg->data == currentCmdIdx){
		//kill process
		char killCmd[100];
		sprintf(killCmd, "kill -2 %d", pidList[currentCmdIdx]);
		system(killCmd);
		currentCmdIdx++;
		ROS_INFO("%d\n",currentCmdIdx);
		idxMutex.unlock();
	}
	else{
		idxMutex.unlock();
	}
}

void rosThread(int thread_idx){
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe("end_topic", 100, endMsgCallback);
	ros::Rate loop_rate(10);

	while(true){
		loop_rate.sleep();
		ros::spinOnce();

		idxMutex.lock();
		if(thread_idx != currentCmdIdx){
			idxMutex.unlock();
			break;
		}
		//cout<<"currentCmdIdx: "<<currentCmdIdx<<'\n';
		idxMutex.unlock();
	}

	sub.shutdown();
}

int main(int argc, char **argv){

	ros::init(argc, argv, "testinit");

	while(currentCmdIdx < MAX_CMD_NUM){
		pid_t c_pid;
		int status;

		c_pid = fork();

		// parent precess
		if(c_pid > 0){

			pid_t wait_pid;

			cout<<"\n\nparent with child"<<c_pid<<"\n\n";

			thread rt = thread(rosThread, currentCmdIdx);

			while(((wait_pid = wait(&status)) == -1) && errno == EINTR);

			rt.join();

			if(wait_pid == -1){
				cout<<"child process killed\n";
			}
			else{
				if(WIFEXITED(status)){
					cout<<"successful exit\n";
				}
				else if(WIFSIGNALED(status)){
					cout<<"wrong exit\n";
				}
			}
		}
		else if (c_pid == 0){
			const char *cmdArgs[MAX_ARG_NUM];

			for(int i = 0; i < cmdList[currentCmdIdx].size(); i++){
				cmdArgs[i] = cmdList[currentCmdIdx][i + 1].c_str();
			}

			cmdArgs[cmdList[currentCmdIdx].size()] = NULL;

			execv(cmdList[currentCmdIdx][0].c_str(), (char **)cmdArgs);
		}
		else{
			perror("failed to fork....\n");
			return 1;
		}
	}

	return 0;
}
