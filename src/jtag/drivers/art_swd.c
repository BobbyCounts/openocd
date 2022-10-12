/***************************************************************************
 *   Copyright (C) 2011 by Richard Uhler                                   *
 *   ruhler@mit.edu                                                        *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifndef _WIN32
#include <sys/un.h>
#include <netdb.h>
#include <netinet/tcp.h>
#endif
#include "helper/system.h"
#include "helper/replacements.h"
#include <jtag/interface.h>
#include "art_bitbang.h"

#include <poll.h>

/* arbitrary limit on host name length: */
#define REMOTE_BITBANG_HOST_MAX 255

static char *remote_bitbang_host;
static char *remote_bitbang_port;

static int remote_bitbang_fd;
struct pollfd pfds;
static uint8_t remote_bitbang_send_buf[512];
static unsigned int remote_bitbang_send_buf_used;

/* Circular buffer. When start == end, the buffer is empty. */
static char remote_bitbang_recv_buf[512];
static unsigned int remote_bitbang_recv_buf_start;
static unsigned int remote_bitbang_recv_buf_end;

static bool remote_bitbang_recv_buf_full(void)
{
	return remote_bitbang_recv_buf_end ==
		((remote_bitbang_recv_buf_start + sizeof(remote_bitbang_recv_buf) - 1) %
		 sizeof(remote_bitbang_recv_buf));
}

static bool remote_bitbang_recv_buf_empty(void)
{
	return remote_bitbang_recv_buf_start == remote_bitbang_recv_buf_end;
}

static unsigned int remote_bitbang_recv_buf_contiguous_available_space(void)
{
	if (remote_bitbang_recv_buf_end >= remote_bitbang_recv_buf_start) {
		unsigned int space = sizeof(remote_bitbang_recv_buf) -
				     remote_bitbang_recv_buf_end;
		if (remote_bitbang_recv_buf_start == 0)
			space -= 1;
		return space;
	} else {
		return remote_bitbang_recv_buf_start -
		       remote_bitbang_recv_buf_end - 1;
	}
}

static int remote_bitbang_flush(void)
{
	if (remote_bitbang_send_buf_used <= 0)
		return ERROR_OK;

	unsigned int offset = 0;
	while (offset < remote_bitbang_send_buf_used) {
	    //set blocking write
	    //socket_block(remote_bitbang_fd);
	    
	    ssize_t written;
	    int status;
	    int timeout_tries = 10;
	    int i;
	    
	    for(i = 0; i < timeout_tries; i ++){
	        status = poll(&pfds, 1, 25);    //poll for write status for 25ms
	        if(status == -1){
	            LOG_ERROR("Error polling the status of the socket");
	            return ERROR_FAIL;
	        }
	        if(pfds.revents & POLLOUT){
                break;
	        }else{
	            LOG_DEBUG("Stall for socket");
	            continue;
	        }
	    }
	      
	    written = write_socket(remote_bitbang_fd, remote_bitbang_send_buf + offset,
									       remote_bitbang_send_buf_used - offset);
		if (written < 0) {
		    log_socket_error("remote_bitbang_putc");
			return ERROR_FAIL;

		    }else
		        break;
	       

		offset += written;
	}

	
	remote_bitbang_send_buf_used = 0;
	return ERROR_OK;
}

enum block_bool {
	NO_BLOCK,
	BLOCK
};

/* Read any incoming data, placing it into the buffer. */
static int remote_bitbang_fill_buf(enum block_bool block)
{
	if (remote_bitbang_recv_buf_empty()) {
		/* If the buffer is empty, reset it to 0 so we get more
		 * contiguous space. */
		remote_bitbang_recv_buf_start = 0;
		remote_bitbang_recv_buf_end = 0;
	}

	if (block == BLOCK) {
		if (remote_bitbang_flush() != ERROR_OK)
			return ERROR_FAIL;
		socket_block(remote_bitbang_fd);
	}

	bool first = true;
	while (!remote_bitbang_recv_buf_full()) {
		unsigned int contiguous_available_space =
				remote_bitbang_recv_buf_contiguous_available_space();
		ssize_t count = read_socket(remote_bitbang_fd,
				remote_bitbang_recv_buf + remote_bitbang_recv_buf_end,
				contiguous_available_space);
		if (first && block == BLOCK)
			socket_nonblock(remote_bitbang_fd);
		first = false;
		if (count > 0) {
			remote_bitbang_recv_buf_end += count;
			if (remote_bitbang_recv_buf_end == sizeof(remote_bitbang_recv_buf))
				remote_bitbang_recv_buf_end = 0;
		} else if (count == 0) {
			return ERROR_OK;
		} else if (count < 0) {
#ifdef _WIN32
			if (WSAGetLastError() == WSAEWOULDBLOCK) {
#else
			if (errno == EAGAIN) {
#endif
				return ERROR_OK;
			} else {
				log_socket_error("remote_bitbang_fill_buf");
				return ERROR_FAIL;
			}
		}
	}

	return ERROR_OK;
}

typedef enum {
	NO_FLUSH,
	FLUSH_SEND_BUF
} flush_bool_t;

static int remote_bitbang_queue(int c, flush_bool_t flush)
{
	remote_bitbang_send_buf[remote_bitbang_send_buf_used++] = c;
	if (flush == FLUSH_SEND_BUF ||
			remote_bitbang_send_buf_used >= ARRAY_SIZE(remote_bitbang_send_buf))
		return remote_bitbang_flush();
	return ERROR_OK;
}

static int remote_bitbang_quit(void)
{
	if (remote_bitbang_queue('Q', FLUSH_SEND_BUF) == ERROR_FAIL)
		return ERROR_FAIL;

	if (close_socket(remote_bitbang_fd) != 0) {
		log_socket_error("close_socket");
		return ERROR_FAIL;
	}

	free(remote_bitbang_host);
	free(remote_bitbang_port);

	LOG_INFO("remote_bitbang interface quit");
	return ERROR_OK;
}

static bb_value_t char_to_int(int c)
{
	switch (c) {
		case '0':
			return BB_LOW;
		case '1':
			return BB_HIGH;
		default:
			remote_bitbang_quit();
			LOG_ERROR("remote_bitbang: invalid read response: %c(%i)", c, c);
			return BB_ERROR;
	}
}

/*
static int remote_bitbang_sample(void)
{
	if (remote_bitbang_fill_buf(NO_BLOCK) != ERROR_OK)
		return ERROR_FAIL;
	assert(!remote_bitbang_recv_buf_full());
	return remote_bitbang_queue('R', FLUSH_SEND_BUF);
}
*/


static bb_value_t remote_bitbang_read_sample(void)
{
	if (remote_bitbang_recv_buf_empty()) {
		if (remote_bitbang_fill_buf(BLOCK) != ERROR_OK)
			return BB_ERROR;
	}
	assert(!remote_bitbang_recv_buf_empty());
	int c = remote_bitbang_recv_buf[remote_bitbang_recv_buf_start];
	remote_bitbang_recv_buf_start =
		(remote_bitbang_recv_buf_start + 1) % sizeof(remote_bitbang_recv_buf);
	return char_to_int(c);
}


static int remote_bitbang_write(int tck, int tms, int tdi)
{
	char c = '0' + ((tck ? 0x4 : 0x0) | (tms ? 0x2 : 0x0) | (tdi ? 0x1 : 0x0));
	return remote_bitbang_queue(c, NO_FLUSH);
}


static int remote_bitbang_reset(int trst, int srst)
{
	char c = 'r' + ((trst ? 0x2 : 0x0) | (srst ? 0x1 : 0x0));
	/* Always flush the send buffer on reset, because the reset call need not be
	 * followed by jtag_execute_queue(). */
	return remote_bitbang_queue(c, NO_FLUSH);
}

static int remote_bitbang_blink(int on)
{
	//char c = on ? 'B' : 'b';
	//return remote_bitbang_queue(c, FLUSH_SEND_BUF);
	return ERROR_OK;
}



static int remote_swd_write(int swclk, int swdio)
{
    //char c = '0' + ((swclk ? 0x4 : 0x0) | (0x2) | (swdio ? 0x1 : 0x0));
	//return remote_bitbang_queue(c, NO_FLUSH);
    return remote_bitbang_write(swclk, 0, swdio);
}



static int remote_swdio_read(void)
{
	if (remote_bitbang_fill_buf(NO_BLOCK) != ERROR_OK)
		return ERROR_FAIL;
	assert(!remote_bitbang_recv_buf_full());
	assert(remote_bitbang_recv_buf_empty());
    remote_bitbang_queue('I', NO_FLUSH);
    bb_value_t result = remote_bitbang_read_sample();
    if(result == BB_LOW)
        return 0;
    else
        return 1;
}

static void remote_swdio_drive(bool is_output){
    char c = is_output ? 'D' : 'd';
    remote_bitbang_queue(c, NO_FLUSH);
}

static struct bitbang_interface remote_bitbang_bitbang = {
	.buf_size = sizeof(remote_bitbang_recv_buf) - 1,
	//.sample = &remote_bitbang_sample,
	//.read_sample = &remote_bitbang_read_sample,
	//.write = &remote_bitbang_write,
	.blink = &remote_bitbang_blink,
	//.swdio_read_reg = remote_swdio_read_reg,
	//.swdio_write_reg = remote_swdio_write_reg
    .swdio_read = remote_swdio_read,
    .swd_write = remote_swd_write,
	.swdio_drive = remote_swdio_drive,
};

static int remote_bitbang_init_tcp(void)
{
	struct addrinfo hints = { .ai_family = AF_UNSPEC, .ai_socktype = SOCK_STREAM };
	struct addrinfo *result, *rp;
	int fd = 0;

	LOG_INFO("Connecting to %s:%s",
			remote_bitbang_host ? remote_bitbang_host : "localhost",
			remote_bitbang_port);

	/* Obtain address(es) matching host/port */
	int s = getaddrinfo(remote_bitbang_host, remote_bitbang_port, &hints, &result);
	if (s != 0) {
		LOG_ERROR("getaddrinfo: %s\n", gai_strerror(s));
		return ERROR_FAIL;
	}

	/* getaddrinfo() returns a list of address structures.
	 Try each address until we successfully connect(2).
	 If socket(2) (or connect(2)) fails, we (close the socket
	 and) try the next address. */

	for (rp = result; rp ; rp = rp->ai_next) {
		fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
		if (fd == -1)
			continue;

		if (connect(fd, rp->ai_addr, rp->ai_addrlen) != -1)
			break; /* Success */

		close(fd);
	}

	/* We work hard to collapse the writes into the minimum number, so when
	 * we write something we want to get it to the other end of the
	 * connection as fast as possible. */
	int one = 1;
	/* On Windows optval has to be a const char *. */
	setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, (const char *)&one, sizeof(one));

	freeaddrinfo(result); /* No longer needed */

	if (!rp) { /* No address succeeded */
		log_socket_error("Failed to connect");
		return ERROR_FAIL;
	}

	return fd;
}

static int remote_bitbang_init_unix(void)
{
	if (!remote_bitbang_host) {
		LOG_ERROR("host/socket not specified");
		return ERROR_FAIL;
	}

	LOG_INFO("Connecting to unix socket %s", remote_bitbang_host);
	int fd = socket(PF_UNIX, SOCK_STREAM, 0);
	if (fd < 0) {
		log_socket_error("socket");
		return ERROR_FAIL;
	}

	struct sockaddr_un addr;
	addr.sun_family = AF_UNIX;
	strncpy(addr.sun_path, remote_bitbang_host, sizeof(addr.sun_path));
	addr.sun_path[sizeof(addr.sun_path)-1] = '\0';

	if (connect(fd, (struct sockaddr *)&addr, sizeof(struct sockaddr_un)) < 0) {
		log_socket_error("connect");
		return ERROR_FAIL;
	}

	return fd;
}

static int remote_bitbang_init(void)
{
	art_bitbang_interface = &remote_bitbang_bitbang;

	remote_bitbang_recv_buf_start = 0;
	remote_bitbang_recv_buf_end = 0;

	LOG_INFO("Initializing remote_bitbang driver");
	if (!remote_bitbang_port)
		remote_bitbang_fd = remote_bitbang_init_unix();
	else
		remote_bitbang_fd = remote_bitbang_init_tcp();

	if (remote_bitbang_fd < 0)
		return remote_bitbang_fd;

	socket_nonblock(remote_bitbang_fd);

    pfds.fd = remote_bitbang_fd;
    pfds.events = POLLOUT;

	LOG_INFO("remote_bitbang driver initialized");
	return ERROR_OK;
}

COMMAND_HANDLER(remote_bitbang_handle_remote_bitbang_port_command)
{
	if (CMD_ARGC == 1) {
		uint16_t port;
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], port);
		free(remote_bitbang_port);
		remote_bitbang_port = port == 0 ? NULL : strdup(CMD_ARGV[0]);
		return ERROR_OK;
	}
	return ERROR_COMMAND_SYNTAX_ERROR;
}

COMMAND_HANDLER(remote_bitbang_handle_remote_bitbang_host_command)
{
	if (CMD_ARGC == 1) {
		free(remote_bitbang_host);
		remote_bitbang_host = strdup(CMD_ARGV[0]);
		return ERROR_OK;
	}
	return ERROR_COMMAND_SYNTAX_ERROR;
}

//
// read command structure
//
typedef struct {
  uint8_t id;
  uint8_t cmd;
  uint32_t data;
} __attribute__ ((packed)) cmd_readreg_t;
typedef struct {
  uint8_t ack;
  uint32_t data;
  uint8_t parity;
} __attribute__ ((packed)) cmd_readreg_resp_t;

//
// write command structure
//
typedef struct {
  uint8_t id;
  uint8_t cmd;
  uint32_t data;
} __attribute__ ((packed)) cmd_writereg_t;
typedef struct {
  uint8_t ack;
} __attribute__ ((packed)) cmd_writereg_resp_t;

static int remote_bitbang_queue_buffer(uint8_t *packet, int num_bytes, flush_bool_t flush)
{
    int i;
    for(i = 0; i < num_bytes; i++){
        remote_bitbang_queue(packet[i], NO_FLUSH);
    }
    remote_bitbang_flush();
    return ERROR_OK;
}

static int remote_bitbang_read_packet(uint8_t *packet, int num_bytes){
	int i;
	for(i = 0; i < num_bytes; i++){	
	    if (remote_bitbang_recv_buf_empty()) {
		    if (remote_bitbang_fill_buf(BLOCK) != ERROR_OK)
			    return ERROR_FAIL;
	    }
	
		assert(!remote_bitbang_recv_buf_empty());
	    packet[i] = remote_bitbang_recv_buf[remote_bitbang_recv_buf_start];
	    remote_bitbang_recv_buf_start =
		    (remote_bitbang_recv_buf_start + 1) % sizeof(remote_bitbang_recv_buf);	
	}
    return ERROR_OK;
}


void queue_swdio_write_reg(uint8_t cmd, uint32_t data) {
	cmd_writereg_t packet;
	//cmd_writereg_resp_t response;
	//int result;

	packet.id = 'W';
	packet.cmd = cmd;
	packet.data = data;
	remote_bitbang_queue_buffer((uint8_t *)&packet, sizeof(packet), FLUSH_SEND_BUF);
}

void queue_swdio_read_reg(uint8_t cmd) {
	cmd_readreg_t packet;
	//cmd_readreg_resp_t response;
	//int result;

	packet.id = 'E';
	packet.cmd = cmd;
	remote_bitbang_queue_buffer((uint8_t *)&packet, sizeof(packet), FLUSH_SEND_BUF);
}

void response_swdio_write_reg(uint8_t *ack) {
	cmd_writereg_resp_t response;
	int result;

	
	result = remote_bitbang_read_packet((uint8_t *)&response, sizeof(response));
	if (result == -1) {
		exit(-1);
	}
	*ack = response.ack;
}

void response_swdio_read_reg(uint8_t *ack, uint32_t *data, uint8_t *parity) {
	cmd_readreg_resp_t response;
	int result;

	result = remote_bitbang_read_packet((uint8_t *)&response, sizeof(response));
	if (result == -1) {
		exit(-1);
	}
	*ack = response.ack;
	*data = response.data;
	*parity = response.parity;
}

void art_swdio_flush(void) {
	//remote_bitbang_flush();
	return;
}

void art_flush_queues(void){
    remote_bitbang_flush();
    remote_bitbang_fill_buf(NO_BLOCK);
    remote_bitbang_recv_buf_start = remote_bitbang_recv_buf_end;
    remote_bitbang_send_buf_used = 0;
    return;
}


static int art_speed(int speed)
{
	return ERROR_OK;
}

static int art_speed_div(int speed, int *khz)
{
	/* I don't think this really matters any. */
	*khz = 1;
	return ERROR_OK;
}

static int art_khz(int khz, int *jtag_speed)
{
	*jtag_speed = 0;
	return ERROR_OK;
}

static const struct command_registration remote_bitbang_command_handlers[] = {
	{
		.name = "art_port",
		.handler = remote_bitbang_handle_remote_bitbang_port_command,
		.mode = COMMAND_CONFIG,
		.help = "Set the port to use to connect to the remote jtag.\n"
			"  if 0 or unset, use unix sockets to connect to the remote jtag.",
		.usage = "port_number",
	},
	{
		.name = "art_host",
		.handler = remote_bitbang_handle_remote_bitbang_host_command,
		.mode = COMMAND_CONFIG,
		.help = "Set the host to use to connect to the remote jtag.\n"
			"  if port is 0 or unset, this is the name of the unix socket to use.",
		.usage = "host_name",
	},
	COMMAND_REGISTRATION_DONE,
};

static const char * const art_swd_transports[] = { "swd", NULL };

struct adapter_driver art_swd_adapter_driver = {
	.name = "art-swd",
	.transports = art_swd_transports,
	.commands = remote_bitbang_command_handlers,

	.init = &remote_bitbang_init,
	.quit = &remote_bitbang_quit,
	.reset = &remote_bitbang_reset,

	.swd_ops = &art_bitbang_swd,
	
    .speed = &art_speed,
	.speed_div = &art_speed_div,
	.khz = &art_khz,
};

