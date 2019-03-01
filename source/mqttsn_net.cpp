/*
 * mqttsn_net.cpp
 *
 *  Created on: May 31, 2018
 *      Author: Dmitry
 */

#include "mqttsn_net.h"

#include "common/code_utils.hpp"

#include <stdlib.h>
#include <string.h>


//=============================================================================
//
//=============================================================================

IPStack::IPStack()
{
    memset(&mSocket, 0, sizeof(mSocket));
    memset(&mPeer, 0, sizeof(mPeer));
}


int IPStack::Socket_error(const char* aString)
{
    (void)aString;
#if 0
    int rc = 0;
    //if (errno != EINTR && errno != EAGAIN && errno != EINPROGRESS && errno != EWOULDBLOCK)
    //{
        if (strcmp(aString, "shutdown") != 0 || (errno != ENOTCONN && errno != ECONNRESET))
        {
            if (errno != EINTR && errno != EAGAIN && errno != EINPROGRESS && errno != EWOULDBLOCK)
            printf("Socket error %s in %s for socket %d\n", strerror(errno), aString, mysock);
            rc = errno;
        }
    //}
    return rc;
#endif
    return 0;
}


int IPStack::connect(const char* hostname, int port)
{
#if 0
    int type = SOCK_STREAM;
    struct sockaddr_in address;
    int rc = -1;
    sa_family_t family = AF_INET;
    struct addrinfo *result = NULL;
    struct addrinfo hints = {0, AF_UNSPEC, SOCK_STREAM, IPPROTO_TCP, 0, NULL, NULL, NULL};

    if ((rc = getaddrinfo(hostname, NULL, &hints, &result)) == 0)
    {
        struct addrinfo* res = result;

        /* prefer ip4 addresses */
        while (res)
        {
            if (res->ai_family == AF_INET)
            {
                result = res;
                break;
            }
            res = res->ai_next;
        }

        if (result->ai_family == AF_INET)
        {
            address.sin_port = htons(port);
            address.sin_family = family = AF_INET;
            address.sin_addr = ((struct sockaddr_in*)(result->ai_addr))->sin_addr;
        }
        else
            rc = -1;

        freeaddrinfo(result);
    }

    if (rc == 0)
    {
        mysock = socket(family, type, 0);
        if (mysock != -1)
        {
            //int opt = 1;

            //if (setsockopt(mysock, SOL_SOCKET, SO_NOSIGPIPE, (void*)&opt, sizeof(opt)) != 0)
            //  printf("Could not set SO_NOSIGPIPE for socket %d", mysock);

            rc = ::connect(mysock, (struct sockaddr*)&address, sizeof(address));
        }
    }
#endif

    int rc = 0;

    otSockAddr sockaddr;
    memset(&sockaddr, 0, sizeof(otSockAddr));
    sockaddr.mPort = 7335;

    //TODO: add errors processing
    otUdpOpen(mInstance, &mSocket, &IPStack::HandleUdpReceive, this);

    otUdpBind(&mSocket, &sockaddr);

    return rc;
}


int IPStack::read(unsigned char* buffer, int len, int timeout_ms)
{
#if 0
    struct timeval interval = {timeout_ms / 1000, (timeout_ms % 1000) * 1000};
    if (interval.tv_sec < 0 || (interval.tv_sec == 0 && interval.tv_usec <= 0))
    {
        interval.tv_sec = 0;
        interval.tv_usec = 100;
    }

    setsockopt(mysock, SOL_SOCKET, SO_RCVTIMEO, (char *)&interval, sizeof(struct timeval));

    int bytes = 0;
    while (bytes < len)
    {
        int rc = ::recv(mysock, &buffer[bytes], (size_t)(len - bytes), 0);
        if (rc == -1)
        {
            if (Socket_error("read") != 0)
            {
                bytes = -1;
                break;
            }
        }
        else
            bytes += rc;
    }

    return bytes;
#endif
    return 0;
}


int IPStack::write(unsigned char* buffer, int len, int timeout)
{
#if 0
    struct timeval tv;

    tv.tv_sec = 0;  /* 30 Secs Timeout */
    tv.tv_usec = timeout * 1000;  // Not init'ing this can cause strange errors

    setsockopt(mysock, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv,sizeof(struct timeval));
    int rc = ::write(mysock, buffer, len);
    //printf("write rc %d\n", rc);
    return rc;
#endif
    otError error = OT_ERROR_NONE;
    otMessage *message;

    VerifyOrExit((message = otUdpNewMessage(mInstance, true)) != NULL, error = OT_ERROR_NO_BUFS);
    SuccessOrExit(error = otMessageSetLength(message, len));
    otMessageWrite(message, 0, buffer, len);
    SuccessOrExit(error = otUdpSend(&mSocket, message, &mPeer));

exit:

    if (error != OT_ERROR_NONE && message != NULL)
    {
        otMessageFree(message);
        len = 0;
    }

    return len;
}


int IPStack::disconnect()
{
    otUdpClose(&mSocket);
    return 0;
}


void IPStack::HandleUdpReceive(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
    static_cast<IPStack *>(aContext)->HandleUdpReceive(aMessage, aMessageInfo);
}


void IPStack::HandleUdpReceive(otMessage *aMessage, const otMessageInfo *aMessageInfo)
{
#if 0
    uint16_t payloadLength = otMessageGetLength(aMessage) - otMessageGetOffset(aMessage);
    char buf[512];

    VerifyOrExit(payloadLength <= sizeof(buf));
    otMessageRead(aMessage, otMessageGetOffset(aMessage), buf, payloadLength);

    if (buf[payloadLength - 1] == '\n')
    {
        buf[--payloadLength] = '\0';
    }

    if (buf[payloadLength - 1] == '\r')
    {
        buf[--payloadLength] = '\0';
    }

    mPeer = *aMessageInfo;

    mInterpreter->ProcessLine(buf, payloadLength, *this);
 #endif
}


//=============================================================================
//
//=============================================================================


Countdown::Countdown()
{

}


Countdown::Countdown(int ms)
{
    countdown_ms(ms);
}


bool Countdown::expired()
{
    struct timeval now, res;
    gettimeofday(&now, NULL);
    timersub(&end_time, &now, &res);
    //printf("left %d ms\n", (res.tv_sec < 0) ? 0 : res.tv_sec * 1000 + res.tv_usec / 1000);
    //if (res.tv_sec > 0 || res.tv_usec > 0)
    //  printf("expired %d %d\n", res.tv_sec, res.tv_usec);
    return res.tv_sec < 0 || (res.tv_sec == 0 && res.tv_usec <= 0);
}


void Countdown::countdown_ms(int ms)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    struct timeval interval = {ms / 1000, (ms % 1000) * 1000};
    //printf("interval %d %d\n", interval.tv_sec, interval.tv_usec);
    timeradd(&now, &interval, &end_time);
}


void Countdown::countdown(int seconds)
{
    struct timeval now;
    gettimeofday(&now, NULL);
    struct timeval interval = {seconds, 0};
    timeradd(&now, &interval, &end_time);
}


int Countdown::left_ms()
{
    struct timeval now, res;
    gettimeofday(&now, NULL);
    timersub(&end_time, &now, &res);
    //printf("left %d ms\n", (res.tv_sec < 0) ? 0 : res.tv_sec * 1000 + res.tv_usec / 1000);
    return (res.tv_sec < 0) ? 0 : res.tv_sec * 1000 + res.tv_usec / 1000;
}


