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
#include <string.h>

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

int main(void)
{
	struct sockaddr_in myaddr, remaddr;
	int fd, i, slen = sizeof(remaddr);
	char buf[BUFLEN];	/* message buffer */
	int recvlen;		/* # bytes in acknowledgement message */
	char *server = "127.0.0.1";	/* change this to use a different server */

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
		/* now receive an acknowledgement from the server */
		recvlen = recvfrom(fd, buf, BUFLEN, 0, (struct sockaddr *)&remaddr, &slen);
		if (recvlen >= 0) {
			buf[recvlen] = 0;	/* expect a printable string - terminate it */
			printf("received message: \"%s\"\n", buf);
		}
	}
	WSACleanup(); //call this for destructor
	// close(fd);
	return 0;
}