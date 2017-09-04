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
#define HAVE_STRUCT_TIMESPECs
#include <string>
#include <sys\timeb.h>
#include <time.h>
#include <windows.h>
//#include "tasksThread.h"


//-------------------------
// Macros
//-------------------------

#undef DEBUG_PRINT


long serverROA::now_ms() {

  ftime(&start);
  long timestamp = clock();
  return timestamp;
}

//ssssssss
bool serverROA::sendTCPpack(char* buffer) {

  if (sendto(sockfd, buffer, strlen(buffer), 0, (struct sockaddr *)&remaddr, addrlen) < 0) {
    perror("sendto error?");
    return 0;
  }
  else return 1;

}


bool serverROA::send_start_pkt(int ID) {

  char* buffer = new char[BUFSIZE];
  ///*optional*/printf("sending msg: , \"%s\", timestamp:%u \n", buffer, now_ms());
  sprintf (buffer, "$START,%d;", ID);
  return sendTCPpack(buffer);
}

bool serverROA::send_end_pkt(int ID) {

  char* buffer = new char[BUFSIZE];
  ///*optional*/printf("sending msg: , \"%s\", timestamp:%u \n", buffer, now_ms());
  sprintf (buffer, "$END,%d;", ID);
  return sendTCPpack(buffer);
}

/*this one ? change name*/
bool serverROA::sendIndividualPoint(struct individualCloud* sendCloud,  uint32_t index) {
  char* buffer = new char[BUFSIZE];
  // std::string sendText = "RGBA is " + std::to_string(sendCloud[2000].rgba);
  // strcpy(buffer, sendText.c_str());
  // buffer[0] = &sendText;
  // buffer = "sending individualCloud struct";
  //uint32_t index_array = 2000;
  ///**/sprintf (buffer, "$START,ROA;");
  ///**/sprintf (buffer, "$START,BODYP;");
  // /**/printf("sending response: , \"%s\", timestamp:%u \n", buffer, now_ms());
  // sendTCPpack(buffer);

  // for (int i = 0; i < max_index_array; i++ ) {
  //ssssssss
  // printf("serverROA ADD send_cloud_pkt: %p \n", sendCloud  );  
  // /*okish*/sprintf (buffer, "$,%u,%f,%f,%f,%u;", (uint32_t)index, (float)11, (float)12, (float)13, (uint32_t)5); //sendCloud[index].rgba);
  // /**/printf("---msgpkt::%s|| timestamp:%u \n", buffer, now_ms());
///**/sprintf (buffer, "$,%u,%f,%f,%f,%u;", *(uint32_t)sendCloud[index].index, *(float)sendCloud[index].x, *(float)sendCloud[index].y, *(float)sendCloud[index].z, *sendCloud[index].rgba);
  /*ok*/sprintf (buffer, "$,%u,%f,%f,%f,%u;", (uint32_t)sendCloud[index].index, sendCloud[index].x, (float)sendCloud[index].y, (float)sendCloud[index].z, sendCloud[index].rgba);
  // /**/printf("sending response \"%s\"\n", buffer);
  // /**/printf("---msgpkt::%s|| timestamp:%u \n", buffer, now_ms());
  if (sendto(sockfd, buffer, strlen(buffer), 0, (struct sockaddr *)&remaddr, addrlen) < 0) {
    perror("sendto error?");
    return 0;
  }
  // }

  ///**/sprintf (buffer, "$END;");
  ///**/printf("sending response: , \"%s\", timestamp:%u \n", buffer, now_ms());
  //sendTCPpack(buffer);


  return 1;
}


bool serverROA::sendIndividualCloud(struct individualCloud* sendCloud) {
  char* buffer = new char[BUFSIZE]; /* receive buffer */
  std::string sendText = "RGBA is " + std::to_string(sendCloud[2000].rgba);
  strcpy(buffer, sendText.c_str());
  // buffer[0] = &sendText;
  // buffer = "sending individualCloud struct";

  // printf("sending response \"%s\"\n", buffer);
  if (sendto(sockfd, buffer, strlen(buffer), 0, (struct sockaddr *)&remaddr, addrlen) < 0) {

    perror("sendto error?");
    return 0;
  }


  return 1;
}


serverROA::serverROA() {

  printf("\nInside serverROA.cpp\n");
  //ScheduleTask* taskHandeler = new ScheduleTask();

  initialize_connection();
  listener();

  // while (1) {
  //   sendIndividualCloud(ROA_individualCloud);
  //   Sleep(1000);
  // }

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