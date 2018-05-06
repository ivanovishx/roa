// =============================================================================
//
//  ROA_CLIENT_VIEWPCL_V2.cpp
//  TCP/UDP  Class to send individualPoint Struct
//
//  Created by Ivan Lozano 8/24/17.
//  For research purposes
//
// =============================================================================


// TODO: find: xxxx here and write on rebuildCloud cloud
// email para sotelo:
// #1 update documento, codigo pictures
// #2 empresa taiwanesa brain
// #3 empresa de datastream elon musk

//////////////////////Start new file

/*void rebuildCloud(individualPoint cloud_client, pcl::PointCloud<pcl::PointXYZRGB>::Ptr* , int status, uint32_t index) {
/*xxxx here//#1 Cloud data-outputs, #2 Cloud outputs to visualizer, #3 index output to write #2.1 decision but eliminate this
	//int index=0;
	//TODO***: add For loop to push in ceros if index is higer
	pcl::PointXYZRGB point;

	if (status == 0) {
		//reset cloud
	}
	else if (status == 1) {
		//add point
		point.x = cloud_client.x;
		point.y = cloud_client.y;
		point.z = cloud_client.z;
		point.rgb = cloud_client.rgba; //*reinterpret_cast<float*>(&rgb);
		printf("index:%u\n",index);
		// pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr2;
		// point_cloud_ptr2->points.push_back(point);
		// *point_cloud_ptr.points.push_back(point);
	}
}
*/

//----------------------------------------
// TODO: after START, set cloud mem to 0 with clearCloud(&point_cloud_ptr)
// TODO: after END, print all the new cloud starting in 0, to 307200 points void printAllPointsinCLoud(individualPoint receivedCloud);
//TODO: add delay in sender END, we are missing some END packets

// printf("point_index:%u\n",point_index);
// printf("-->index2:%u x:%f y:%f z:%f rgba:%u \n", cloud_spliter.index, cloud_spliter.x, cloud_spliter.y, cloud_spliter.z, cloud_spliter.rgba);

// printf("index:%u x:%f y:%f z:%f rgba:%u \n", cloud_spliter.index, cloud_spliter.x, cloud_spliter.y, cloud_spliter.z, cloud_spliter.rgba);


// void printAllPointsinCLoud(individualPoint receivedCloud);


//////////////////////END new file



#include "stdafx.h"

#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <stdint.h>
/*--------PCL LIBRARIES: --------*/
//#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
//TIME
#include <sys\timeb.h>
#include <time.h>
#include <windows.h>


//#ifdef WIN32
//add ws2_32.lib to linker on VC++
#include <io.h>
#include <winsock2.h>
#include <ws2tcpip.h>


//#else
//#include <unistd.h>
//#include <sys/types.h>
//#include <sys/socket.h>
//#include <netinet/in.h>
//#include <arpa/inet.h>
//#include <netdb.h>
//#endif // WIN32



#define SERVICE_PORT	2310
#define BUFLEN 2048
#define MSGS 5	/* number of messages to send */

typedef struct individualPoint {
	uint32_t index = 0;
	float x = 0;
	float y = 0;
	float z = 0;
	uint32_t rgba = 0x000000;
} typeindividualPoint;

struct sockaddr_in myaddr, remaddr;
int object_id = -1;
int fd, i, slen = sizeof(remaddr);
int callViewer1time = 0;
int numPointsFrame = 0;
int lastIndexPointFrame = 0;
int actualIndexPointFrame = 0;
int resetIndexEnd = 0;
char buf[BUFLEN];	/* message buffer */
int recvlen;		/* # bytes in acknowledgement message */
char *server = "127.0.0.1";	/* change this to use a different server */
// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
// pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

// TIMERS
uint32_t timerCondition;
/*------FUNCTIONS DECLARATION------*/
void split(std::string& s, char c, std::vector<std::string>& v);
void set_point_packet(individualPoint* _individualPoint, individualPoint cloud_spliter, uint32_t point_index);
void split_point_packet(char* bufin, individualPoint*  cloud_spliter, uint32_t* point_index);
bool flushSocket();
void writeSocket();
char* readSocket();
void confirmGetSTART();
void confirmGetPOINT(int point_index);
void confirmGetEND();
int getID(char* bufin);

// void rebuildCloud(individualPoint* cloud_client, pcl::PointCloud<pcl::PointXYZ>::ConstPtr* cloud_ROA, int status, uint32_t index);
void clearCloud(individualPoint* Matrix_cloud_ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr * point_cloud_ptr);
// void clearCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr* point_cloud_ptr);
void rebuildCloud(individualPoint cloud_client, pcl::PointCloud<pcl::PointXYZRGB>::Ptr* point_cloud_ptr, int status, uint32_t index);

void printAllPointsinCLoud(individualPoint* receivedCloud);
// void printAllPointsinCLoud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr* point_cloud_ptr);
void fillArrayCloud(individualPoint cloud_spliter, individualPoint* array_points, int index_numPointsFrame);
void pushPointsToCloud(individualPoint* cloud_client,	pcl::PointCloud<pcl::PointXYZRGB>::Ptr* point_cloud_ptr);
// boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
/*------Thread function and variables------*/
bool update = false;
// std::mutex Mutex1;
boost::mutex Mutex1;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
void visualizer_thread();
inline uint64_t now_ms();
void msleep(uint64_t useconds);


using namespace std;

int main(void) {

	/*------CREATE CLOUD OBJS------*/
	// pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	// pcl::PointCloud<pcl::PointXYZRGB>::Ptr reset_point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	point_cloud_ptr->width = (int)307200;
	point_cloud_ptr->height = 1;
	// pcl::PointXYZ basic_point;

	/*------POINT DATA PACKET VARIABLES------*/
	struct individualPoint* ROA_cloud = new individualPoint[307200];
	struct individualPoint cloud_spliter;
	uint32_t point_index = 0;

	/*------CREATE SOCKET------*/
	WSADATA Data;
	WSAStartup(MAKEWORD(2, 2), &Data);//just for windows
	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
		printf("socket created\n");

	/* bind it to all local addresses and pick any port number */
	memset((char *)&myaddr, 0, sizeof(myaddr));
	myaddr.sin_family = AF_INET;
	myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	myaddr.sin_port = htons(0);
	int bind_status = 1;// ::bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr));

	if (bind_status < 0) {
		perror("bind failed");
		return 0;
	}

	memset((char *)&remaddr, 0, sizeof(remaddr));
	remaddr.sin_family = AF_INET;
	remaddr.sin_port = htons(SERVICE_PORT);
	if (inet_pton(AF_INET, server, &remaddr.sin_addr) == 0) {
		fprintf(stderr, "inet_aton() failed\n");
		exit(1);
	}

	//HANDSHAKE::
	// for (i = 0; i < MSGS; i++) {
	printf("Sending packet %d to %s port %d\n", i, server, SERVICE_PORT);
	sprintf(buf, "This is packet %d CODE:0xA185FFFF", i);
	if (sendto(fd, buf, strlen(buf), 0, (struct sockaddr *)&remaddr, slen) == -1) {
		perror("sendto");
		exit(1);
	}
	// }


	printf("\nListening comming Cloud Packets:\n");
	char* bufin = new char[BUFLEN];
	int siezeMsg = 0;
	int corruptedCounter = 0;

	/*------STARTING VIEWER THREAD------*/
	boost::thread workerThread(visualizer_thread);//set the visualizer_thread function running the thread
	workerThread.detach();



	/*------LOOP------*/
	// while (true) {

	// 	Mutex1.lock();
	// 	viewer->updatePointCloud (point_cloud_ptr, "sample cloud");
	// 	viewer->spinOnce (1, true);
	// 	Mutex1.unlock();
	// }

	// memset(bufin, 0, sizeof(bufin));

	timerCondition = now_ms();
	update = false;
	bool oneRun = true;
	uint64_t timerInprogress = 0;
	pcl::PointXYZRGB point;

	while (1) {

		/*thread*/
		// boost::mutex::scoped_lock updateLock(Mutex1);
		/*thread*/
		// printf("while loop--\n");
		bufin = readSocket();


		// if (now_ms() - timerCondition > 5000) {
		// printf("----------BUFFIN:\n");
		// printf(bufin);
		// printf("\n");
		// }
		// printf("while $.\n");
		// msleep(1000);
		// if(bufin[0] != 78)
		// printf("bufin:%c ::%d\n", bufin[0],bufin[0]);

		// if (bufin[0] == '$' && oneRun) {
		if (bufin[0] == '$') {

			if (bufin[1] == 'S' && bufin[2] == 'T' && bufin[3] == 'A') {

				numPointsFrame = 0;
				printf("\nRECEIVED: #1-----START!!!");

				// if (object_id = 2) {
				// 	point_cloud_ptr->points.clear();
				// }

				//clearCloud(ROA_cloud, &point_cloud_ptr);
				actualIndexPointFrame = 0;
				lastIndexPointFrame = 0;
				object_id = getID(bufin);
				printf(" ID:%d TM: %u\n",  object_id, now_ms());
				confirmGetSTART();
			}
			// else if ((resetIndexEnd) || (bufin[1] == 'E' && bufin[2] == 'N' && bufin[3] == 'D')) {
			else if ( (bufin[1] == 'E' && bufin[2] == 'N' && bufin[3] == 'D')) {
				printf("RECEIVED: #2-----Number of Points on Cloud Frame:%d \n", numPointsFrame);
				printf("RECEIVED: #3-----END!!! ID:%d TM: %u\n",  object_id, now_ms());

				actualIndexPointFrame = 0;
				lastIndexPointFrame = 0;
				confirmGetEND();
				// flushSocket();
				//printAllPointsinCLoud(ROA_cloud);
				// TODO: xxxx
				// printAllPointsinCLoud(&point_cloud_ptr);
				//printAllPointsinCLoud(&cloud_spliter);


				// pushPointsToCloud(ROA_cloud, &point_cloud_ptr);

////////////////
				/*
				if (object_id = 1){
				point_cloud_ptr->points.clear();
				}
				/**/
				printf("TEST Point #4 \n");
				// pcl::PointXYZRGB point;

				int j = 0;
				timerInprogress = 0;
				update = true;

				for (int i = 0; i < 307200; i++ ) {
					// if (timerInprogress < now_ms()) {printf("TEST Point #5 END in progress...\n"); timerInprogress = now_ms() + 10;}
					if (i == ROA_cloud[j].index) {
						point.x = ROA_cloud[j].x ;
						point.y = ROA_cloud[j].y ;
						point.z = ROA_cloud[j].z ;

						if (object_id == 1) { //ROA ID
							point.rgb = (uint32_t)16777215;//white ok
						}

						else if (object_id == 2) { //BODYP ID
							point.rgb = (uint32_t)1113872; //green ok
						}

						else {				//unknow object
							point.rgb = (uint32_t)16727100;//red ok
						}
						// point.rgb = ROA_cloud[j].rgba;
						j++;
						// }
					}
					else {
						point.x = 0;
						point.y = 0;
						point.z = 0;
						point.rgb = 0;
					}
					//TODO sync this with thread xxxxxxxx
					// cloud->points.push_back (point);
					// printf("#6: i:%d j:%d ", i,j);
					;;;;;;;;;;
					// msleep(1);
					point_cloud_ptr->points.push_back (point);

					// printf("#7\n");
					// point_cloud_ptr->points.push_back (point);
					// printf("TEST point 2\n");

				}
				// printf("TEST point 1\n");


////////////////
				//call visualizer:
				// printf("TEST point 1.1\n");
				update = false;
				// Mutex1.lock();

				cloud = point_cloud_ptr;

				// Mutex1.unlock()
				/*threat test...*/
				// boost::mutex::scoped_lock updateLock(Mutex1);
				// updateLock.try_lock();
				// updateLock.unlock();
				/*...threat test*/
				// printf("^^ THREAD:main  STATUS:_lock\n");


				// Mutex1.lock();
				// printf("TEST point 2.1\n");


				// TODO: implement another thread ot nonblocking method
				// if (!viewer->wasStopped ())
				// {
				// viewer->spinOnce (100);
				// boost::this_thread::sleep (boost::posix_time::microseconds (100000));
				////////////////////-------------old ok/////////////
				/////coming from real_sense_viewer.cpp
				// if (point_cloud_ptr)
				// { 
				// boost::mutex::scoped_lock lock (new_cloud_mutex_);
				// if (!viewer->updatePointCloud (point_cloud_ptr, "cloud"))
				// {
				// viewer->addPointCloud (point_cloud_ptr, "cloud");
				//	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
				// }
				//// displaySettings ();
				//// last_cloud_ = point_cloud_ptr;
				//point_cloud_ptr.reset ();
				// }


				// viewer->addPointCloud (point_cloud_ptr, "cloud");
				// viewer->updatePointCloud (point_cloud_ptr, "cloud");


				uint32_t timerViewer = now_ms();
				// while (1) {
				// for (int m = 0; m < 20; m++) {
				// if (((int)now_ms() - (int)timertestViewer) > 1000) {
				// 	timerViewer = now_ms();
				// viewer->spinOnce (1, true);
				// 	// viewer->spinOnce (100);
				// boost::this_thread::sleep (boost::posix_time::microseconds (100000));
				// }//for end
				// }
				// }
				// }

				//ixxxx uncomment this please
				// viewer->removeAllPointClouds(0); xxx delete cloud ?
				// printf("TEST point 2.1\n");

				numPointsFrame = 0;

				// printf("TEST point 3...\n");
				// printf("^^ THREAD:main  STATUS:unlock\n");
				// Mutex1.unlock();


				// if (workerThread.joinable()) {
				// 	printf("^^ THREAD:main  STATUS:detach first time\n");
				// 	workerThread.detach();
				// }
				// else {
				// 	printf("^^xx THREAD:main  STATUS:NO  detach first time\n");
				// }
				update = true;
				// updateLock.unlock();
				// updateLock.lock();
				printf("END msg...\n");
				oneRun = false;
			}//END msg
			else if ( (bufin[1] == 'P' && bufin[2] == 'N' && bufin[3] == 'T')) {
				// printf(".");
				if (timerInprogress < now_ms()) {printf("Getting cloud in progress...\n"); timerInprogress = now_ms() + 1000;}

				struct individualPoint cloud_spliter2;
				// lastIndexPointFrame = cloud_spliter.index;

				split_point_packet(bufin, &cloud_spliter, &point_index);//#1 raw data, #2 Cloud data-outputs, #3 index output to write

				// printf("-->index2:%u x:%f y:%f z:%f rgba:%u \n", cloud_spliter.index, cloud_spliter.x, cloud_spliter.y, cloud_spliter.z, cloud_spliter.rgba);
				//#1 Cloud data-outputs, #2 Cloud outputs to visualizer, #3 index output to write #2.1 decision but eliminate this

				//printf("---point_index:%u\n", point_index);

				confirmGetPOINT((int)point_index);
				fillArrayCloud(cloud_spliter, ROA_cloud, numPointsFrame);
				// void fillArrayCloud(individualPoint* cloud_spliter, individualPoint* array_points, int index_numPointsFrame);
				numPointsFrame++;
				// rebuildCloud( cloud_spliter,  &ROA_cloud, 1, point_index);
				// rebuildCloud( cloud_spliter,  &point_cloud_ptr, 1, point_index);

				// printf("numPointsFrame:%d, lastIndexPointFrame:%d, actualIndexPointFrame:%u,\n", numPointsFrame, lastIndexPointFrame, actualIndexPointFrame );

				// actualIndexPointFrame = point_index;
				//actualIndexPointFrame = cloud_spliter.index;

				// if ((actualIndexPointFrame - lastIndexPointFrame) < 0 ) {
				// 	printf("numPointsFrame:%d, lastIndexPointFrame:%d, actualIndexPointFrame:%u,\n", numPointsFrame, lastIndexPointFrame, actualIndexPointFrame );
				// 	printf("DIFF EXEPT:%d\n", actualIndexPointFrame - lastIndexPointFrame);
				// 	resetIndexEnd = 0;
				// }
				// rebuildCloud(ROA_individualPoint,  point_cloud_ptr, int status, uint32_t index);

				///*xxxxx*/point_cloud_ptr->points.push_back(pcl::PointXYZRGBA(cloud_spliter.x, cloud_spliter.y, cloud_spliter.z, cloud_spliter.rgba ));

				// pcl::PointCloud<pcl::PointXYZ> cloud;
				// cloud.push_back (pcl::PointXYZ (rand (), rand (), rand ()));

			}
			else {
				printf("Received corrupted packet %d\n", corruptedCounter);
				corruptedCounter ++;
			}

		}
		else {
			msleep(10);
		}


		// printf("OUT OF IF BUFF[$]\n");
		// // memset(bufin, 0, sizeof(bufin));
		// printf("OUT OF IF BUFF[$] pass memset\n");
	}

	// workerThread.join();
	// writeSocket();
	WSACleanup(); //call this for destructor
	// close(fd);
	return 0;
}

void set_point_packet(individualPoint * _individualPoint, individualPoint cloud_spliter, uint32_t point_index) {
	_individualPoint[point_index].index = point_index;
	_individualPoint[point_index].x = cloud_spliter.x;
	_individualPoint[point_index].y = cloud_spliter.y;
	_individualPoint[point_index].z = cloud_spliter.z;
	_individualPoint[point_index].rgba = cloud_spliter.rgba;
}

void split(std::string & s, char c, std::vector<std::string>& v) {
	string::size_type i = 0;
	string::size_type j = s.find(c);
	while (j != string::npos) {
		v.push_back(s.substr(i, j - i));
		i = ++j;
		j = s.find(c, j);
		if (j == string::npos)
			v.push_back(s.substr(i, s.length()));
	}
}

void split_point_packet(char* bufin, individualPoint * cloud_spliter, uint32_t* point_index) {
	std::string string_vector(sizeof(typeindividualPoint), 0);
	string_vector = std::string(bufin);
	std::vector<std::string> v;
	split(string_vector, ',', v);
	cloud_spliter->index = std::stoul(v[1]);
	cloud_spliter->x = std::stof(v[2]);
	cloud_spliter->y = std::stof(v[3]);
	cloud_spliter->z = std::stof(v[4]);
	cloud_spliter->rgba = std::stoul(v[5]);
	//printf("index:%u x:%f y:%f z:%f rgba:%u \n", cloud_spliter.index, cloud_spliter.x, cloud_spliter.y, cloud_spliter.z, cloud_spliter.rgba);
	*point_index = cloud_spliter->index;
}

bool flushSocket() {
	char* bufin = new char[BUFLEN];
	memset(&bufin, 0, sizeof(bufin));
	do {
		recvlen = recvfrom(fd, buf, BUFLEN, 0, (struct sockaddr *)&remaddr, &slen);
		printf("recvlen %d\n", recvlen);
		if (recvlen >= 0) {
			buf[recvlen] = 0;	/* expect a printable string - terminate it */
			bufin = buf;
		}
		printf("FLUSHING INPUT BUFFER:%s\n", bufin);
		// return bufin;
	} while (recvlen >= 0);
	printf("-----FLUSHING DONE!!! TM: %u\n", now_ms());

	return 1;

}
void writeSocket() {
	i = 9;
	printf("Sending packet %d to %s port %d\n", i, server, SERVICE_PORT);
	sprintf(buf, "This is packet %d CODE:0xA185FFFF", i);
	if (sendto(fd, buf, strlen(buf), 0, (struct sockaddr *)&remaddr, slen) == -1) {
		perror("sendto");
		exit(1);
	}
}

char* readSocket() {
	char* bufin = new char[BUFLEN];
	memset(&bufin, 0, sizeof(bufin));
	recvlen = recvfrom(fd, buf, BUFLEN, 0, (struct sockaddr *)&remaddr, &slen);
	if (recvlen >= 0) {
		buf[recvlen] = 0;	/* expect a printable string - terminate it */
		bufin = buf;
	}
	else {
		bufin = "NO_DATA_ON_BUFFER";
	}
	return bufin;
}
/*--------CONFIRMATION START, END:-------*/
void confirmGetSTART() {
	i = 9;
	printf("SENDING: START confirmation to ROA Server %d to %s port %d\n", i, server, SERVICE_PORT);
	sprintf(buf, "$startok\n");
	if (sendto(fd, buf, strlen(buf), 0, (struct sockaddr *)&remaddr, slen) == -1) {
		perror("sendto");
		exit(1);
	}
}

void confirmGetPOINT(int point_index) {

	//printf("Sending POINT confirmation to ROA Server %u to %s port %d\n", point_index, server, SERVICE_PORT);
	// if (now_ms() - timerCondition > 10000) {
	// 	printf(buf);
	// }

	sprintf(buf, "%d", point_index);
	if (sendto(fd, buf, strlen(buf), 0, (struct sockaddr *)&remaddr, slen) == -1) {
		perror("sendto");
		exit(1);
	}
}

void confirmGetEND() {
	i = 9;
	printf("SENDING: END confirmation to ROA Server %d to %s port %d\n", i, server, SERVICE_PORT);
	sprintf(buf, "$endok\n");
	if (sendto(fd, buf, strlen(buf), 0, (struct sockaddr *)&remaddr, slen) == -1) {
		perror("sendto");
		exit(1);
	}
}

int getID(char* bufin) {
	std::string string_vector(sizeof(BUFLEN), 0);
	string_vector = std::string(bufin);
	std::vector<std::string> v;
	split(string_vector, ',', v);
	// printf("%c\n", bufin[7]);
	int ID1 = std::stof(v[1]);
	// printf("---ID1:%d \n", ID1);
	return ID1;

}

/*--------CREATE CLOUD:-------*/
void clearCloud(individualPoint * Matrix_cloud_ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr * point_cloud_ptr) {
	memset(Matrix_cloud_ptr, 0, sizeof(Matrix_cloud_ptr));
	memset(&point_cloud_ptr, 0, sizeof(point_cloud_ptr));
	printf("---------- ClearCloud DONE after START;\n");
}
void rebuildCloud(individualPoint cloud_client, individualPoint * _cloud, int status, uint32_t index) {
// void rebuildCloud(individualPoint cloud_client, pcl::PointCloud<pcl::PointXYZRGB>::Ptr * point_cloud_ptr, int status, uint32_t index) {
	/*xxxx here*///#1 Cloud data-outputs, #2 Cloud outputs to visualizer, #3 index output to write #2.1 decision but eliminate this
	//int index=0;
	//TODO***: add For loop to push in ceros if index is higer
	pcl::PointXYZRGB point;

	if (status == 0) {
		//reset cloud
	}
	else if (status == 1) {
		//add point
		point.x = cloud_client.x;
		point.y = cloud_client.y;
		point.z = cloud_client.z;
		point.rgb = cloud_client.rgba; //*reinterpret_cast<float*>(&rgb);
		// printf("index:%u\n", index);

		printf("index:%u x:%f y:%f z:%f rgba:%u \n", cloud_client.index, cloud_client.x, cloud_client.y, cloud_client.z, cloud_client.rgba);

		// <-- ok:::working here -->

		// pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr2;
		// point_cloud_ptr2->points.push_back(point);
		// *point_cloud_ptr.points.push_back(point);
	}
}

// (pcl::PointCloud<pcl::PointXYZRGB>::Ptr* point_cloud_ptr)


// void printAllPointsinCLoud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr* point_cloud_ptr)
void printAllPointsinCLoud(individualPoint * receivedCloud)
{
	printf("-----------------PRINTING REBUILD STARTS-------------------------\n");
	// float sumRow = receivedCloud->index[i], receivedCloud->x[i], receivedCloud->y[i], receivedCloud->z[i], receivedCloud->rgba[i]
	for (int i = 0; i < 500; i++ ) {
		// for (int i = 0; i < 307200; i++ ) {
//		if(sumRow>0)
		printf("-->i:%d index:%u x:%f y:%f z:%f rgba:%u \n", i, receivedCloud[i].index, receivedCloud[i].x, receivedCloud[i].y, receivedCloud[i].z, receivedCloud[i].rgba);
		// printf("-->i:%d index:%u x:%f y:%f z:%f rgba:%u \n", i, point_cloud_ptr[i].index, point_cloud_ptr[i].x, point_cloud_ptr[i].y, point_cloud_ptr[i].z, point_cloud_ptr[i].rgba);

	}
	printf("----------------- PRINTING REBUILD ENDS-------------------------\n");
}


void fillArrayCloud(individualPoint cloud_spliter, individualPoint * array_points, int index_numPointsFrame) {


	array_points[index_numPointsFrame].index = cloud_spliter.index;
	array_points[index_numPointsFrame].x = cloud_spliter.x;
	array_points[index_numPointsFrame].y = cloud_spliter.y;
	array_points[index_numPointsFrame].z = cloud_spliter.z;
	array_points[index_numPointsFrame].rgba = cloud_spliter.rgba;


}

void pushPointsToCloud(individualPoint * cloud_client,	pcl::PointCloud<pcl::PointXYZRGB>::Ptr * point_cloud_ptr) {
	//#1 input_matrix #2 Final output to Viewer
	pcl::PointXYZRGB point;
	int j = 0;
	for (int i = 0; i < 307200; i++ ) {

		if (i == cloud_client[j].index) {
			// float sumRow = receivedCloud->index[i], receivedCloud->x[i], receivedCloud->y[i], receivedCloud->z[i], receivedCloud->rgba[i]
			// if (sumRow != 0) {
			point.x = cloud_client[j].x;
			// point.x = cloud_client.x;
			point.y = cloud_client[j].y;
			point.z = cloud_client[j].z;
			point.rgb = cloud_client[j].rgba;
			j++;
			// }

		}
		else {
			point.x = 0;
			point.y = 0;
			point.z = 0;
			point.rgb = 0;

		}

		// point_cloud_ptr->points.push_back (point);
		// *point_cloud_ptr->points.push_back (point);

	}
}

// void msleep(int time){}



/*--------VISUALIZER FUNCTIONS:-------*/


// https://stackoverflow.com/questions/9003239/stream-of-cloud-point-visualization-using-pcl
// this https://github.com/mpp/LCD/blob/e57b601376e559e6e3d98a905a77eefc488a13b5/MOSAIC/pclvisualizerthread.cpp
// https://github.com/keshavchintamani/triniBot/blob/master/pclstuff/src/pclstuff.cpp
// https://github.com/robotology/grasp/blob/0db531799370f55dea08347a015c4b4f99a60b8a/object-reconstruction/src/visThread.cpp
/*
void visualizer_thread() {//tttttt
// TODO: compare with file: ROA_CLIENT_VIEWPCL_V2_BAK3_nothread_yet.cpp
// TODO: especialemtne en la secuencia de viewer ;)
printf("-- THREAD:visualizer_thread  STATUS:Starting...\n");
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_color_handler(cloud, 230, 20, 0); //red
// boost::mutex::scoped_lock updateLock(Mutex1);


while (!viewer->wasStopped ()) {
	// Mutex1.unlock();
	if (update) {
		printf("-- THREAD:visualizer_thread  STATUS:lock && update true\n");
		Mutex1.lock();
		if (!viewer->updatePointCloud(cloud, "sample cloud")) {
			// pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
			// viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
			viewer->addPointCloud(cloud, cloud_color_handler, "sample cloud");
		}
		update = false;
		printf("-- THREAD:visualizer_thread  STATUS:unlock\n");
		Mutex1.unlock();
	}

	else {
		if (Mutex1.try_lock()) {
			printf("-- THREAD:visualizer_thread  STATUS:viewer->spinOnce (100)\n");
			// viewer->spinOnce (1, true);
			// viewer_.spinOnce (1, true);
			// -->viewer->spinOnce ();
			// viewer->spinOnce (100);
			Mutex1.unlock();
			printf("-- THREAD:visualizer_thread  STATUS:unlock && viewer->OUT of spinOnce (100)\n");
		}
		else {
			printf("-- THREAD:visualizer_thread  STATUS:try_lock FAIL \n");
		}

		// printf("-- THREAD:visualizer_thread  STATUS:update==false\n");
	}
}
printf("-- THREAD:visualizer_thread  STATUS:....End of thread\n");
}
*/
void visualizer_thread() {//ttttt2
	printf("--Starting visualizer_thread...\n");
	//CLOUD POINTERS
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	// pcl::PointCloud<pcl::PointXYZRGB>::Ptr reset_point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	point_cloud_ptr->width = (int)307200;
	point_cloud_ptr->height = 1;
	// pcl::PointXYZ basic_point;

	cloud = point_cloud_ptr;

	// rgbVis::---------
	//VIEWER
	viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	viewer->setCameraFieldOfView (1.785398); // approximately 45 degrees
	viewer->setCameraPosition (0, 0.1, 0.3, 0, 0, 1, 0, 1, 0);

	// point_cloud_ptr = cloud;
	while (true) {
		// populate the point cloud
		// Mutex1.lock();
		// viewer->removePointCloud("cloud");
		// viewer->addPointCloud(cloud, "cloud", 1);
		if(update == false){			
		viewer->updatePointCloud (cloud, "sample cloud");
		}
		// viewer->addPointCloudNormals<pcl::PointNormal, pcl::PointNormal> (cloud, cloud, 25, 0.15, "normals");
		viewer->spinOnce (1, true);
		// Mutex1.unlock();
	}

	while (true) {
		if (viewer->wasStopped()) break;
		// Mutex1.lock();
		// viewer->spinOnce();
		// Mutex1.unlock();

		printf("visualizer_thread: %d\n", i);
		i++;
	}

}

struct timeb start; //now_ms

inline uint64_t now_ms() {
	ftime(&start);
	long timestamp = clock();
	return timestamp;
}

// inline uint64_t now_ms() {
// 	uint64_t timestamp = pcl::getTime () * 1.0e+3;
// 	return timestamp;
// }

void msleep(uint64_t useconds) {
	uint64_t goal = now_ms() + useconds;
	while (goal > now_ms());
}

