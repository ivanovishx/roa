// =============================================================================
//
//  ROA_client.cpp
//  TCP/UDP  Class to send individualCloud Struct
//
//  Created by Ivan Lozano 8/24/17.
//  For research purposes
//
// =============================================================================
#include "stdafx.h"



#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <stdint.h>

#ifdef WIN32
//add ws2_32.lib to linker on VC++
#include <io.h>
#include <winsock2.h>
#include <ws2tcpip.h>

#else
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#endif // WIN32

#define SERVICE_PORT	2310
#define BUFLEN 2048
#define MSGS 5	/* number of messages to send */


typedef struct individualCloud
{
	uint32_t index = 0;
	float x = 0;
	float y = 0;
	float z = 0;
	uint32_t rgba = 0x000000;
	// uint32_t rgba = 0;

} typeIndividualCloud;

struct sockaddr_in myaddr, remaddr;
int fd, i, slen = sizeof(remaddr);
char buf[BUFLEN];	/* message buffer */
int recvlen;		/* # bytes in acknowledgement message */
char *server = "127.0.0.1";	/* change this to use a different server */

/*------FUNCTIONS DECLARATION------*/
// void split( char* s, char c, char* v );
void split( std::string& s, char c, std::vector<std::string>& v );
// void split( string s, char c, vector<string> v );
void set_point_packet(individualCloud* _individualCloud, individualCloud cloud_spliter, uint32_t point_index );
void split_point_packet(char* bufin, individualCloud  cloud_spliter, uint32_t* point_index  );
void writeSocket();
char* readSocket();


using namespace std;

int main(void)
{

	/* create a socket */
	WSADATA Data;
	WSAStartup(MAKEWORD(2, 2), &Data);//just for windows
	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
		printf("socket created\n");

	/* bind it to all local addresses and pick any port number */

	memset((char *)&myaddr, 0, sizeof(myaddr));
	myaddr.sin_family = AF_INET;
	myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	myaddr.sin_port = htons(0);

	if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
		perror("bind failed");
		return 0;
	}

	/* now define remaddr, the address to whom we want to send messages */
	/* For convenience, the host address is expressed as a numeric IP address */
	/* that we will convert to a binary format via inet_aton */

	memset((char *)&remaddr, 0, sizeof(remaddr));
	remaddr.sin_family = AF_INET;
	remaddr.sin_port = htons(SERVICE_PORT);
	if (inet_pton(AF_INET, server, &remaddr.sin_addr) == 0) {
//	if (inet_aton(server, &remaddr.sin_addr) == 0) {
		fprintf(stderr, "inet_aton() failed\n");
		exit(1);
	}

	/* now let's send the messages */

	for (i = 0; i < MSGS; i++) {
		printf("Sending packet %d to %s port %d\n", i, server, SERVICE_PORT);
		sprintf(buf, "This is packet %d CODE:0xA185FFFF", i);
		if (sendto(fd, buf, strlen(buf), 0, (struct sockaddr *)&remaddr, slen) == -1) {
			perror("sendto");
			exit(1);
		}
		// 	/* now receive an acknowledgement from the server */
		// 	recvlen = recvfrom(fd, buf, BUFLEN, 0, (struct sockaddr *)&remaddr, &slen);
		// 	if (recvlen >= 0) {
		// 		buf[recvlen] = 0;	/* expect a printable string - terminate it */
		// 		printf("received message: \"%s\"\n", buf);
		// 	}
	}


	//Listen new messages from ROA_SERVER sending individualCloud:
	printf("\nListening comming Cloud Packets:\n");

	char* bufin = new char[BUFLEN];
	int siezeMsg = 0;
	//------POINT DATA PACKET VARIABLES------
	struct individualCloud* ROA_individualCloud = new individualCloud[307200];
	struct individualCloud cloud_spliter;
	uint32_t point_index = 0;

//wwwww
	// memset (&ROA_individualCloud, 0, sizeof (ROA_individualCloud));//reset frame

	while (1) {


		bufin = readSocket();

		if (bufin[0] == '$') {
			if (bufin[1] == 'S' && bufin[2] == 'T' && bufin[3] == 'A') {
				printf("START!!!\n");
			}
			else if ( bufin[1] == 'E' && bufin[2] == 'N' && bufin[3] == 'D') {
				printf("END!!!\n");
			}
			else {

				split_point_packet( bufin,  cloud_spliter, &point_index);
				// set_point_packet(ROA_individualCloud, cloud_spliter, point_index);


				//**/printf("index: %u, color: %u \n", point_index, cloud_spliter.rgba);

				// printf("index:%u %f %f %u \n", ROA_individualCloud[300].index, ROA_individualCloud[300].x, ROA_individualCloud[300].y, ROA_individualCloud[300].rgba );
				// printf("index:%u %f %f %u \n", ROA_individualCloud[20].index, ROA_individualCloud[20].x, ROA_individualCloud[20].y, ROA_individualCloud[20].rgba );
			}
		}
		// printf("--Value: %s\n", bufin);

	}
	// writeSocket();
	//END
	WSACleanup(); //call this for destructor
	// close(fd);
	return 0;
}

void set_point_packet(individualCloud* _individualCloud, individualCloud cloud_spliter, uint32_t point_index  ) {

	_individualCloud[point_index].index = point_index;
	_individualCloud[point_index].x =  cloud_spliter.x;
	_individualCloud[point_index].y = cloud_spliter.y;
	_individualCloud[point_index].z = cloud_spliter.z;
	_individualCloud[point_index].rgba = cloud_spliter.rgba;


	/*point_index = (uint32_t)20;
	_individualCloud[point_index].index = point_index+1;
	_individualCloud[point_index].x =  3;
	_individualCloud[point_index].y = 45;
	_individualCloud[point_index].z = cloud_spliter.z;
	_individualCloud[point_index].rgba = 0xA185FFFF;*/

}


void split( std::string& s, char c, std::vector<std::string>& v) {
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

void split_point_packet(char* bufin, individualCloud cloud_spliter,  uint32_t* point_index  ) {

	std::string string_vector (sizeof(typeIndividualCloud), 0);

	string_vector = std::string(bufin);


	// printf("Before Vector\n");
	std::vector<std::string> v;
	split(string_vector, ',', v);


	printf("myvector stores:%d\n", int(v.size()) );
	// cloud_spliter.index = stoul(v[0]);
	cloud_spliter.index = std::stoul(v[1]);
	cloud_spliter.x = std::stof(v[2]);
	cloud_spliter.y = std::stof(v[3]);
	cloud_spliter.z = std::stof(v[4]);
	// printf("5\n");
	cloud_spliter.rgba = std::stoul(v[5]);
	// cloud_spliter.rgba = std::stoul(v[5]);
	// printf("6\n");
	printf("index:%u x:%f y:%f z:%f rgba:%u \n", cloud_spliter.index, cloud_spliter.x, cloud_spliter.y, cloud_spliter.z, cloud_spliter.rgba );


	// printf("After Vector\n");
	*point_index = cloud_spliter.index;


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
	memset (&bufin, 0, sizeof (bufin));
	recvlen = recvfrom(fd, buf, BUFLEN, 0, (struct sockaddr *)&remaddr, &slen);
	if (recvlen >= 0) {
		buf[recvlen] = 0;	/* expect a printable string - terminate it */
		bufin = buf;
		// printf("received message: \"%s\"\n", buf);
	}
	// else sprintf (bufin, "--ERROR IN DATA--\n");
	return bufin;
}