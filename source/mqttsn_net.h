/*
 * mqttsn_net.h
 *
 *  Created on: May 31, 2018
 *      Author: Dmitry
 */

#ifndef MQTTSN_NET_H_
#define MQTTSN_NET_H_

#include "openthread-core-config.h"

//#include <openthread/types.h>
#include <openthread/udp.h>

#include <sys/time.h>

class IPStack
{
public:
    IPStack();

    int Socket_error(const char* aString);

    int connect(const char* hostname, int port);

    int read(unsigned char* buffer, int len, int timeout_ms);

    int write(unsigned char* buffer, int len, int timeout);

    int disconnect();

private:

    static void HandleUdpReceive(void *aContext, otMessage *aMessage, const otMessageInfo *aMessageInfo);
    void HandleUdpReceive(otMessage *aMessage, const otMessageInfo *aMessageInfo);

    otUdpSocket mSocket;
    otMessageInfo mPeer;
    otInstance *mInstance;

};


class Countdown
{
public:
    Countdown();

    Countdown(int ms);

    bool expired();

    void countdown_ms(int ms);

    void countdown(int seconds);

    int left_ms();

private:

    struct timeval end_time;

};

#endif /* MQTTSN_NET_H_ */
