// =============================================================================
//
//  ROA_client.cpp
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

/*void rebuildCloud(individualPoint cloud_client, pcl::PointCloud<pcl::PointXYZRGB>::Ptr* point_cloud_ptr, int status, uint32_t index) {
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
int fd, i, slen = sizeof(remaddr);
int callViewer1time = 0;
int numPointsFrame = 0;
int lastIndexPointFrame = 0;
int actualIndexPointFrame = 0;
int resetIndexEnd = 0;
char buf[BUFLEN];	/* message buffer */
int recvlen;		/* # bytes in acknowledgement message */
char *server = "127.0.0.1";	/* change this to use a different server */

/*------FUNCTIONS DECLARATION------*/
void split(std::string& s, char c, std::vector<std::string>& v);
void set_point_packet(individualPoint* _individualPoint, individualPoint cloud_spliter, uint32_t point_index);
void split_point_packet(char* bufin, individualPoint*  cloud_spliter, uint32_t* point_index);
void writeSocket();
char* readSocket();
void confirmGetSTART();
void confirmGetEND();

// void rebuildCloud(individualPoint* cloud_client, pcl::PointCloud<pcl::PointXYZ>::ConstPtr* cloud_ROA, int status, uint32_t index);
void clearCloud(individualPoint* Matrix_cloud_ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr * point_cloud_ptr);
// void clearCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr* point_cloud_ptr);
void rebuildCloud(individualPoint cloud_client, pcl::PointCloud<pcl::PointXYZRGB>::Ptr* point_cloud_ptr, int status, uint32_t index);

void printAllPointsinCLoud(individualPoint* receivedCloud);
// void printAllPointsinCLoud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr* point_cloud_ptr);
void fillArrayCloud(individualPoint cloud_spliter, individualPoint* array_points, int index_numPointsFrame);
void pushPointsToCloud(individualPoint* cloud_client,	pcl::PointCloud<pcl::PointXYZRGB>::Ptr* point_cloud_ptr);
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

using namespace std;

int main(void) {

	/*------CREATE CLOUD OBJS------*/
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	// struct individualPoint* ROA_individualPoint = new struct individualPoint[307200];
	//pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	point_cloud_ptr->width = (int)307200;
	point_cloud_ptr->height = 1;
	pcl::PointXYZ basic_point;

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

	for (i = 0; i < MSGS; i++) {
		printf("Sending packet %d to %s port %d\n", i, server, SERVICE_PORT);
		sprintf(buf, "This is packet %d CODE:0xA185FFFF", i);
		if (sendto(fd, buf, strlen(buf), 0, (struct sockaddr *)&remaddr, slen) == -1) {
			perror("sendto");
			exit(1);
		}
	}

	printf("\nListening comming Cloud Packets:\n");
	char* bufin = new char[BUFLEN];
	int siezeMsg = 0;

	/*------LOOP------*/


	while (1) {
		bufin = readSocket();
		if (bufin[0] == '$') {
			if (bufin[1] == 'S' && bufin[2] == 'T' && bufin[3] == 'A') {
				//if (()||(bufin[1] == 'S' && bufin[2] == 'T' && bufin[3] == 'A')) {
				numPointsFrame = 0;
				printf("#1-----START!!!\n");
				clearCloud(ROA_cloud, &point_cloud_ptr);
				actualIndexPointFrame = 0;
				lastIndexPointFrame = 0;
				confirmGetSTART();


			}
			// else if ((resetIndexEnd) || (bufin[1] == 'E' && bufin[2] == 'N' && bufin[3] == 'D')) {
			else if ( (bufin[1] == 'E' && bufin[2] == 'N' && bufin[3] == 'D')) {
				printf("#2-----Number of Points on Cloud Frame:%d \n", numPointsFrame);
				printf("#3-----END!!!\n");

				actualIndexPointFrame = 0;
				lastIndexPointFrame = 0;
				confirmGetEND();
				// TODO: xxxx
				printAllPointsinCLoud(ROA_cloud);
				// printAllPointsinCLoud(&point_cloud_ptr);
				//printAllPointsinCLoud(&cloud_spliter);


				// pushPointsToCloud(ROA_cloud, &point_cloud_ptr);

////////////////
				pcl::PointXYZRGB point;
				int j = 0;
				for (int i = 0; i < 307200; i++ ) {

					if (i == ROA_cloud[j].index) {
						// float sumRow = receivedCloud->index[i], receivedCloud->x[i], receivedCloud->y[i], receivedCloud->z[i], receivedCloud->rgba[i]
						// if (sumRow != 0) {
						point.x = ROA_cloud[j].x  * 10;
						// point.x = cloud_client.x;
						point.y = ROA_cloud[j].y  * 10;
						point.z = ROA_cloud[j].z  * 10;
						point.rgb = ROA_cloud[j].rgba;
						j++;
						// }

					}
					else {
						point.x = 0;
						point.y = 0;
						point.z = 0;
						point.rgb = 0;

					}


					point_cloud_ptr->points.push_back (point);

				}



////////////////
				//call visualizer:

				if (numPointsFrame > 500 && callViewer1time == 0) {
					callViewer1time = 1;
					viewer = rgbVis(point_cloud_ptr);
					// }
					// else {

					while (!viewer->wasStopped ())
					{
						viewer->spinOnce (100);
						boost::this_thread::sleep (boost::posix_time::microseconds (100000));
					}
				}

				// TODO: implement another thread ot nonblocking method
				// if (!viewer->wasStopped ())
				// {
				// 	viewer->spinOnce (100);
				// 	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
				// }


				numPointsFrame = 0;

			}
			else {
				struct individualPoint cloud_spliter2;
				// lastIndexPointFrame = cloud_spliter.index;

				split_point_packet(bufin, &cloud_spliter, &point_index);//#1 raw data, #2 Cloud data-outputs, #3 index output to write

				// printf("-->index2:%u x:%f y:%f z:%f rgba:%u \n", cloud_spliter.index, cloud_spliter.x, cloud_spliter.y, cloud_spliter.z, cloud_spliter.rgba);
				//#1 Cloud data-outputs, #2 Cloud outputs to visualizer, #3 index output to write #2.1 decision but eliminate this


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

		}
	}

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
	return bufin;
}
/*--------CONFIRMATION START, END:-------*/
void confirmGetSTART() {
	i = 9;
	printf("Sending START confirmation to ROA Server %d to %s port %d\n", i, server, SERVICE_PORT);
	sprintf(buf, "$startok");
	if (sendto(fd, buf, strlen(buf), 0, (struct sockaddr *)&remaddr, slen) == -1) {
		perror("sendto");
		exit(1);
	}
}
void confirmGetEND() {
	i = 9;
	printf("Sending END confirmation to ROA Server %d to %s port %d\n", i, server, SERVICE_PORT);
	sprintf(buf, "$endok");
	if (sendto(fd, buf, strlen(buf), 0, (struct sockaddr *)&remaddr, slen) == -1) {
		perror("sendto");
		exit(1);
	}
}

/*--------CREATE CLOUD:-------*/
void clearCloud(individualPoint* Matrix_cloud_ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr * point_cloud_ptr) {
	memset(Matrix_cloud_ptr, 0, sizeof(Matrix_cloud_ptr));
	memset(&point_cloud_ptr, 0, sizeof(point_cloud_ptr));
	printf("---------- ClearCloud DONE after START;\n");
}
void rebuildCloud(individualPoint cloud_client, individualPoint* _cloud, int status, uint32_t index) {
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


void fillArrayCloud(individualPoint cloud_spliter, individualPoint* array_points, int index_numPointsFrame) {


	array_points[index_numPointsFrame].index = cloud_spliter.index;
	array_points[index_numPointsFrame].x = cloud_spliter.x;
	array_points[index_numPointsFrame].y = cloud_spliter.y;
	array_points[index_numPointsFrame].z = cloud_spliter.z;
	array_points[index_numPointsFrame].rgba = cloud_spliter.rgba;


}

void pushPointsToCloud(individualPoint* cloud_client,	pcl::PointCloud<pcl::PointXYZRGB>::Ptr* point_cloud_ptr) {
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

/*--------VISUALIZER FUNCTIONS:-------*/

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	// --------------------------------------------
	// ---Open 3D viewer and add point cloud RGBA--
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}
