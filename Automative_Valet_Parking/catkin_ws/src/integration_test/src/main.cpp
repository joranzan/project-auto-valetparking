#include <unistd.h>
#include <iostream>
#include <sys/wait.h>
#include <errno.h>
#include <cstdio>

using namespace std;


int main(){

	pid_t c_pid;
	int status;

	c_pid = fork();

	// parent precess
	if(c_pid > 0){
		pid_t wait_pid;

		cout<<"parent process with child "<<c_pid<<'\n';

		while(((wait_pid = wait(&status)) == -1) && errno == EINTR);

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

		cout<<"done with parent...\n";
	}
	else if (c_pid == 0){

		// system("/opt/ros/melodic/setup.sh");
		// system("~/catkin_ws/devel/setup.sh");

		cout<<execl("/opt/ros/melodic/bin/roslaunch", "roslaunch","ssafy_1", "talker_listener_1.launch",(char*) 0)<<'\n';

		// cout<<execl("launch.sh","launch.sh",(char *)0);

		cout<<"child process\n";

		pid_t new_pid;
		int new_status;

		new_pid = fork();

		if(new_pid == 0){
			cout<<execl("/opt/ros/melodic/bin/roslaunch","ssafy_1", "talker_listener_1.launch",(char*) 0)<<'\n';
		}
		else if (new_pid > 0){
			cout<<"new process: "<<new_pid<<'\n';

			pid_t new_wait_pid;

			char cmd[100];

			sprintf(cmd, "kill -9 %d", new_pid);

			cout<<cmd<<'\n';

			while(((new_wait_pid = wait(&new_status)) == -1) && errno == EINTR);

			cout<<"done with child...\n";
		}
	}
	else{
		perror("failed to fork....\n");
		return 1;
	}

	return 0;
}
