// Copyright (c) 2014-2019 FRC Team 3512. All Rights Reserved.

#include "logging/LogServerSink.hpp"

#ifdef __VXWORKS__
#include <netinet/in.h>
#include <netinet/sctp.h>
#include <selectLib.h>
#include <sockLib.h>
#include <socket.h>
#include <tipc/tipc.h>
#include <types.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <cstdio>
#include <cstring>

using namespace frc3512;

LogServerSink::~LogServerSink() {
    if (m_listensd >= 0) {
        close(m_listensd);
    }
}

void LogServerSink::Log(LogEvent event) {
    std::list<int>::iterator cur;
    int ok;

    for (auto it = m_connections.begin(); it != m_connections.end();) {
        cur = it;
        it++;

        ok = send(*cur, event.ToFormattedString().c_str(),
                  event.ToFormattedString().length(), 0);
        if (ok != static_cast<ssize_t>(event.ToFormattedString().length())) {
            close(*cur);
            m_connections.erase(cur);
        }
    }
}

int LogServerSink::StartServer(uint16_t port) {
    m_listensd = TcpListen(port);
    if (m_listensd < 0) return -1;

    /* Ignore SIGPIPE */
    signal(SIGPIPE, SIG_IGN);

    return 0;
}

int LogServerSink::AcceptConnectionBlocking() {
    socklen_t addrlen;
    struct sockaddr_in addr;
    int sd;

    addrlen = sizeof(struct sockaddr_in);
#ifdef __VXWORKS__
    sd = accept(m_listensd, (struct sockaddr*)&addr,
                reinterpret_cast<int*>(&addrlen));
#else
    sd = accept(m_listensd, (struct sockaddr*)&addr,
                reinterpret_cast<socklen_t*>(&addrlen));
#endif
    if (sd < 0) return -1;
    m_connections.push_back(sd);

    return 0;
}

int LogServerSink::AcceptConnectionNonBlocking() {
    int ok;
    struct timeval timeout;
    fd_set readfds;
    int accepted;

    accepted = 1;
    while (accepted) {
        FD_ZERO(&readfds);
        FD_SET(m_listensd, &readfds);
        timeout.tv_sec = 0;
        timeout.tv_usec = 0;
        ok = select(m_listensd + 1, &readfds, nullptr, nullptr, &timeout);
        if (ok < 0) return -1;
        accepted = 0;
        if (FD_ISSET(m_listensd, &readfds)) {
            if (AcceptConnectionBlocking() == -1) return -1;
            accepted = 1;
        }
    }

    return 0;
}

int LogServerSink::Acceptor(bool blocking) {
    if (blocking) {
        return AcceptConnectionBlocking();
    } else {
        return AcceptConnectionNonBlocking();
    }
}

/**
 * Listens on a specified port and returns the listener file descriptor, or -1
 * on error.
 */
int LogServerSink::TcpListen(uint16_t port) {
    int sd;
    int error;
    struct sockaddr_in addr;

    // Create a socket
    sd = socket(AF_INET, SOCK_STREAM, 0);
    if (sd < 1) return -1;

    // Bind the socket to the address and port we want to listen on.
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;
    error = bind(sd, (struct sockaddr*)&addr, sizeof(struct sockaddr_in));
    if (error != 0) {
        close(sd);
        return -1;
    }

    // Start the socket listening with a backlog of ten connections.
    error = listen(sd, 10);
    if (error != 0) {
        close(sd);
        return -1;
    }

    return sd;
}
