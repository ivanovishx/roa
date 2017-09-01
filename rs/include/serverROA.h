// =============================================================================
//
//  server_ROA.h
//  TCP/UDP  Class to send individualCloud Struct
//
//  Created by Ivan Lozano 8/24/17.
//  For research purposes
//
// =============================================================================
#ifndef __server_ROA_H__
#define __server_ROA_H__

//#include <tasksThread.h>

// #define WIN32//this is defined by Visual Basic Compiler
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys\timeb.h>

#ifdef WIN32
#define HAVE_STRUCT_TIMESPEC
//#include "pthread.h"
// #include <thread>
#include <stdint.h>
#include <io.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>//must to be after winsock2.h to avoid redifinicion problems
#define HAVE_STRUCT_TIMESPEC //must to be before include pthread

#else
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#endif // WIN32

// Thread:
//#include <pthread.h>
// HANDLE Mutex;


//Socket:
static char ServerIPPointerHub[15] = "127.0.0.1";

#define ROA_TX_PORT        2310
#define ROA_TX_RECEIVER    "localhost"

//#define RECEIVE_THREAD_MAX_COUNT 2
//#define RECEIVE_THREAD_0 0
//#define RECEIVE_THREAD_1 1
#define BUFSIZE 2048
// #define RECEIVER_QUEUE_SIZE 4
// namespace server{


class serverROA//cGcsReceiver
{

private:

	// pktFormat_t rPacketBuf[RECEIVER_QUEUE_SIZE];
	// void create_header(uint8_t cmd);
	int   thread_state, sockfd, portno, recvlen, loop, newsockfd;
	int msgcnt = 0;			/* count # of messages we received */
	// cGcsPkt* gcspkt;
	struct  hostent *server;
	//pthread_t thread_id[RECEIVE_THREAD_MAX_COUNT];
	// thread thread_id;
	// pthread_t thread_id;
	WSADATA Data;
	//int thread_state[RECEIVE_THREAD_MAX_COUNT];
	enum eReceiveThreadState {RECEIVE_THREAD_INIT, RECEIVE_THREAD_RUN, RECEIVE_THREAD_EXIT};

	//int sockfd, newsockfd, portno;
	socklen_t addrlen = sizeof(remaddr);		/* length of addresses */
	// socklen_t clilen;
	// uint8_t buffer;//[256];
	struct sockaddr_in myaddr;	/* our address */
	struct sockaddr_in remaddr;	/* remote address */
	struct hostent *hostp; /* client host info */
	char *hostaddrp; /* dotted decimal host addr string */
	int optval; /* flag value for setsockopt */


	struct timeb start; //now_ms
	// pthread_mutex_t count_mutex;
	int write_index;
	int read_index;
	int count;

	const int WIDTH = 640;
	const int HEIGHT = 480;
	const uint32_t SIZE = 307200;

public:

	typedef struct individualCloud
	{
		uint32_t index;
		float x = 0;
		float y = 0;
		float z = 0;
		uint32_t rgba = 0xFF00FF44;
		// uint32_t rgba = 0;

	} typeIndividualCloud;

	// server::individualCloud* ROA_individualCloud2 = new server::individualCloud[307200];
	struct individualCloud* ROA_individualCloud = new individualCloud[307200];
	struct individualCloud* BODYP_individualCloud = new individualCloud[307200];

	serverROA();
	~serverROA();
	// transmitPacket(cGcsPkt* packet);
	long now_ms();
	bool sendTCPpack(char* buffer);
	bool send_start_pkt();
	bool send_end_pkt();
	bool sendIndividualPoint( individualCloud* sendCloud);
	bool sendIndividualCloud( individualCloud* sendCloud);
	// uint8_t* receivePacket(pktFormat_t* packet);
	int initialize_connection(void);
	//static void* threadEntryPoint(void* p);
	void listener(void);
	void* main_thread_entry(void* arg);
	int get_buffer_occupancy(void);
	// void write_into_rPacketBuf(pktFormat_t);
	// pktFormat_t* read_from_rPacketBuf(void);

	static void* threadEntryPoint(void* p)
	{
		printf("gcs_receiver::threadEntrypoint\n");
		static_cast<serverROA*>(p)->main_thread_entry(p);
		return NULL;

	}



};

// }//end namespace server::
#endif //__server_ROA_H__