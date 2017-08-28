// =============================================================================
//
//  server_ROA.h
//  TCP/UDP  Class to send individualCloud Struct
//
//  Created by Ivan Lozano 8/24/17.
//  For research purposes
//
// =============================================================================
//use format from netpipe

#include "serverROA.h"
#include <stdio.h>
#include <stdint.h>
#include <windows.h>
#include <string>
//#include "tasksThread.h"


//-------------------------
// Macros
//-------------------------

#undef DEBUG_PRINT

// #define DEBUG_FATAL(M, ...) {fprintf(stderr,M,##__VA_ARGS__);fflush(stdout);}
// #define printf(M, ...) {fprintf(stderr,M,##__VA_ARGS__);fflush(stdout);}

// #if 0
// #define DEBUG_PRINT(M, ...)
// #define DEBUG_ERROR(M, ...)
// #define DEBUG_LOG(M, ...)
// #else
// #define DEBUG_PRINT(M, ...) {fprintf(stderr,M,##__VA_ARGS__);fflush(stdout);}
// #define DEBUG_ERROR(M, ...) {fprintf(stderr,M,##__VA_ARGS__);fflush(stdout);}
// #define DEBUG_LOG(M, ...)   {fprintf(stderr,M,##__VA_ARGS__);fflush(stdout);}
// #endif
// #define XDEBUG_PRINT(M, ...)
// #define XDEBUG_ERROR(M, ...)
// #define XDEBUG_LOG(M, ...)
// #define Xprintf(M, ...)

//ssssssss
// bool serverROA::sendIndividualCloud(individualCloud* sendCloud) {
bool serverROA::sendIndividualCloud(struct individualCloud* sendCloud) {

  char* buffer = new char[BUFSIZE]; /* receive buffer */

  std::string sendText = "RGBA is " + std::to_string(sendCloud[2000].rgba);
strcpy(buffer, sendText.c_str());
  // buffer[0] = &sendText; 
  // buffer = "sending individualCloud struct"; 

  printf("sending response \"%s\"\n", buffer);
  if (sendto(sockfd, buffer, strlen(buffer), 0, (struct sockaddr *)&remaddr, addrlen) < 0) {

    perror("sendto error?");
    return 0;
  }


  return 1;
}


serverROA::serverROA() {

  // loop = 1;
  // count = read_index=write_index=0;

  printf("\nInside serverROA.cpp\n");
  //ScheduleTask* taskHandeler = new ScheduleTask();

  initialize_connection();
  listener();



  // while (1) {
  //   sendIndividualCloud(ROA_individualCloud);
  //   Sleep(1000);
  // }

//create protocol packet class or send PCD---------------
  // gcspkt = new cGcsPkt();
  // if (gcspkt == NULL)
  // {
  // DEBUG_ERROR("<serverROA> Memory allocation failed \n");
  // }
//create thread ------------------------
  // int err = pthread_create(&(thread_id[RECEIVE_THREAD_0]), NULL, serverROA::threadEntryPoint, (void *)this);
  // int err = pthread_create(&(thread_id), NULL, serverROA::threadEntryPoint, (void *)this);
  // if (err != 0) {
  //   printf("FATAL ERROR:: Thread not created %s\n", strerror(err));
  // }
  // printf("SYSTEM MSSG: ServerROA Thread created.\n");
}

void serverROA::listener() {
  /* now loop, receiving data and printing what we received */
  char* buffer = new char[BUFSIZE]; /* receive buffer */
  /* now loop, receiving data and printing what we received */
  // while (1) {
  for (int i = 0; i < 4; i++) {
    printf("waiting on port %d\n", ROA_TX_PORT);
    recvlen = recvfrom(sockfd, buffer, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
    if (recvlen > 0) {
      buffer[recvlen] = 0;
      printf("received message: \"%s\" (%d bytes)\n", buffer, recvlen);
    }
    else
      printf("uh oh - something went wrong!\n");
    // sprintf(buffer, "ack %d", msgcnt++);
    printf("sending response \"%s\"\n", buffer);
    if (sendto(sockfd, buffer, strlen(buffer), 0, (struct sockaddr *)&remaddr, addrlen) < 0)
      perror("sendto");
  }
}



void* serverROA::main_thread_entry(void* arg)
{

  printf("gcs_receiver::main_thread_entry\n ");
  return NULL;

//   //int tid = RECEIVE_THREAD_0;
//   thread_state = RECEIVE_THREAD_INIT;
// while(loop){
//     switch (thread_state)
//     {
//     case RECEIVE_THREAD_INIT:
//     {

//       printf("gcs_receiver :: init\n");
//       initialize_connection();
//       printf("befor state jump\n");

//       //receive
//       thread_state = RECEIVE_THREAD_RUN;

//     }

//     break;

//     case RECEIVE_THREAD_RUN:
//     {
//       clilen = sizeof(cli_addr);
//       while (true)
//       {
//         printf("gcs_receiver::inside RECEIVE thread run\n")
//         uint8_t *buffer;
//         buffer =((uint8_t*)&(gcspkt->packet));
//         n = recvfrom(sockfd, buffer, FULL_PACKET_SIZE, 0, (struct sockaddr *)&cli_addr, &clilen);
//         if(n>0)
//         {
//           write_into_rPacketBuf(gcspkt->packet);

//         }
//         else
//         {
//           DEBUG_ERROR("ERROR from recvfrom");
//         }

//        //gethostbyaddr: determine who sent the datagram

//         hostp = gethostbyaddr((const char *)&cli_addr.sin_addr.s_addr, sizeof(cli_addr.sin_addr.s_addr), AF_INET);

//         if (hostp == NULL)
//           DEBUG_ERROR("ERROR on gethostbyaddr");
//         hostaddrp = inet_ntoa(cli_addr.sin_addr);
//         if (hostaddrp == NULL)
//           DEBUG_ERROR("ERROR on inet_ntoa\n");
//         printf("server received datagram from %s (%s)\n", hostp->h_name, hostaddrp);
//       }

//       thread_state=RECEIVE_THREAD_EXIT;

//     }
//       break;

//     case RECEIVE_THREAD_EXIT:
//       printf("RECEIVE_THREAD_EXIT\n");
//           loop = 0;
//       break;


//     }

//   }
}

int serverROA::get_buffer_occupancy(void)
{
  return count;
}

// void serverROA::write_into_rPacketBuf(pktFormat_t wPacketVariable)
// {
// pthread_mutex_lock(&count_mutex);

// if(count >= RECEIVER_QUEUE_SIZE)
// {
//   printf("error %d\n",count);
//   pthread_mutex_unlock(&count_mutex);
//   printf("\nERROR:-----------> exiting buffer size\n");fflush(stdout);
// }
// count++;
// printf("count %d  wI %d\n",count,write_index);
// rPacketBuf[write_index] = wPacketVariable;

// write_index=((write_index+1)%RECEIVER_QUEUE_SIZE);
// pthread_mutex_unlock(&count_mutex);

// }

// pktFormat_t* serverROA::read_from_rPacketBuf(void)
// {
//   int r=0;
//   if(count>0)
//   {
//     pthread_mutex_lock(&count_mutex);
//     count--;
//     r = read_index;
//     read_index=((read_index+1)%RECEIVER_QUEUE_SIZE);
//     pthread_mutex_unlock(&count_mutex);
//     return  rPacketBuf + r;
//   }
//   else
//     return NULL;


// }

int serverROA::initialize_connection()
{

  printf("ROA_SERVER::initialize_connection\n");
  //Create a socket
  WSAStartup(MAKEWORD(2, 2), &Data);//just for windows
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
    perror("cannot create socket\n");
    return 0;
  }
  memset((char *) &myaddr, 0, sizeof(myaddr));
  //portno = atoi(s);
  portno = ROA_TX_PORT;
  myaddr.sin_family = AF_INET;
  myaddr.sin_addr.s_addr = INADDR_ANY;
  // myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  myaddr.sin_port = htons(portno);

  //Bind
  if ( bind(sockfd , (struct sockaddr *)&myaddr , sizeof(myaddr)) < 0)
  {
    printf("Bind failed ") ;

  }
  printf("Bind Socket done: listening RAPID clients on port:%d\n", portno);
  // clilen = sizeof(cli_addr);

  printf("initialize done\n");

}


serverROA::~serverROA()
{
  WSACleanup(); //call this for destructor
  printf("Server socket closed\n");
  // close(sockfd);
  // pthread_join(thread_id,NULL);
}





// uint8_t* serverROA::receivePacket(pktFormat_t* packet)
// {
//   // read

// }
