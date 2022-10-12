/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
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

/* 2014-12: Addition of the SWD protocol support is based on the initial work
 * by Paul Fertser and modifications by Jean-Christian de Rivaz. */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "art_bitbang.h"
#include <jtag/interface.h>
#include <jtag/commands.h>

/**
 * Function bitbang_stableclocks
 * issues a number of clock cycles while staying in a stable state.
 * Because the TMS value required to stay in the RESET state is a 1, whereas
 * the TMS value required to stay in any of the other stable states is a 0,
 * this function checks the current stable state to decide on the value of TMS
 * to use.
 */

struct bitbang_interface *art_bitbang_interface;

/* DANGER!!!! clock absolutely *MUST* be 0 in idle or reset won't work!
 *
 * Set this to 1 and str912 reset halt will fail.
 *
 * If someone can submit a patch with an explanation it will be greatly
 * appreciated, but as far as I can tell (ØH) DCLK is generated upon
 * clk = 0 in TAP_IDLE. Good luck deducing that from the ARM documentation!
 * The ARM documentation uses the term "DCLK is asserted while in the TAP_IDLE
 * state". With hardware there is no such thing as *while* in a state. There
 * are only edges. So clk => 0 is in fact a very subtle state transition that
 * happens *while* in the TAP_IDLE state. "#&¤"#¤&"#&"#&
 *
 * For "reset halt" the last thing that happens before srst is asserted
 * is that the breakpoint is set up. If DCLK is not wiggled one last
 * time before the reset, then the breakpoint is not set up and
 * "reset halt" will fail to halt.
 *
 */
#define CLOCK_IDLE() 0

void transaction_queue_init(void);

static int queued_retval;

static void art_queue_swd_write_reg(uint8_t cmd, uint32_t data, uint32_t ap_delay_clk);

static int bitbang_swd_init(void)

{
	LOG_INFO("bitbang_swd_init");
	transaction_queue_init();
	return ERROR_OK;
}

static void bitbang_swd_exchange(bool rnw, uint8_t buf[], unsigned int offset, unsigned int bit_cnt)
{
	LOG_DEBUG("bitbang_swd_exchange");

	if (art_bitbang_interface->blink) {
		/* FIXME: we should manage errors */
		art_bitbang_interface->blink(1);
	}

	for (unsigned int i = offset; i < bit_cnt + offset; i++) {
		int bytec = i/8;
		int bcval = 1 << (i % 8);
		int swdio = !rnw && (buf[bytec] & bcval);

		art_bitbang_interface->swd_write(0, swdio);
		if (rnw && buf) {
			if (art_bitbang_interface->swdio_read()){
				buf[bytec] |= bcval;
			}else
				buf[bytec] &= ~bcval;
		}

		art_bitbang_interface->swd_write(1, swdio);
	}


}

static int bitbang_swd_switch_seq(enum swd_special_seq seq)
{
	LOG_DEBUG("bitbang_swd_switch_seq");

	switch (seq) {
	case LINE_RESET:
		LOG_DEBUG("SWD line reset");
		bitbang_swd_exchange(false, (uint8_t *)swd_seq_line_reset, 0, swd_seq_line_reset_len);
		break;
	case JTAG_TO_SWD:
		LOG_DEBUG("JTAG-to-SWD");
		bitbang_swd_exchange(false, (uint8_t *)swd_seq_jtag_to_swd, 0, swd_seq_jtag_to_swd_len);
		break;
	case JTAG_TO_DORMANT:
		LOG_DEBUG("JTAG-to-DORMANT");
		bitbang_swd_exchange(false, (uint8_t *)swd_seq_jtag_to_dormant, 0, swd_seq_jtag_to_dormant_len);
		break;
	case SWD_TO_JTAG:
		LOG_DEBUG("SWD-to-JTAG");
		bitbang_swd_exchange(false, (uint8_t *)swd_seq_swd_to_jtag, 0, swd_seq_swd_to_jtag_len);
		break;
	case SWD_TO_DORMANT:
		LOG_DEBUG("SWD-to-DORMANT");
		bitbang_swd_exchange(false, (uint8_t *)swd_seq_swd_to_dormant, 0, swd_seq_swd_to_dormant_len);
		break;
	case DORMANT_TO_SWD:
		LOG_DEBUG("DORMANT-to-SWD");
		bitbang_swd_exchange(false, (uint8_t *)swd_seq_dormant_to_swd, 0, swd_seq_dormant_to_swd_len);
		break;
	case DORMANT_TO_JTAG:
		LOG_DEBUG("DORMANT-to-JTAG");
		bitbang_swd_exchange(false, (uint8_t *)swd_seq_dormant_to_jtag, 0, swd_seq_dormant_to_jtag_len);
		break;
	default:
		LOG_ERROR("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}


static void swd_clear_sticky_errors(void)
{
	art_queue_swd_write_reg(swd_cmd(false,  false, DP_ABORT),
		STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR, 0);
}



//
//
// transaction_queue
//
//
static struct transaction_queue_entry {
	uint8_t read;
	uint32_t *dst;
	uint32_t cmd;
	//uint8_t trn_ack_data_parity_trn[DIV_ROUND_UP(4 + 3 + 32 + 1 + 4, 8)];
} *transaction_queue;
static int transaction_queue_length;
static int transaction_queue_allocated;

void transaction_queue_init(void) {
	transaction_queue_allocated = 10000;
	transaction_queue = malloc(transaction_queue_allocated * sizeof(*transaction_queue));
	transaction_queue_length = 0;
}

void transaction_queue_quit(void) {
	free(transaction_queue);	
}

void art_swdio_flush(void);
void art_flush_queues(void);
void response_swdio_write_reg(uint8_t *ack);
void response_swdio_read_reg(uint8_t *ack, uint32_t *data, uint8_t *parity);



static int bitbang_swd_run_queue(void)
{
	int i;
	uint8_t ack;
	uint32_t data;
	uint8_t parity;
	LOG_DEBUG("bitbang_queue");
	//LOG_INFO("++++++++run_queue, %d", transaction_queue_length);
	/* A transaction must be followed by another transaction or at least 8 idle cycles to
	 * ensure that data is clocked through the AP. */
	bitbang_swd_exchange(true, NULL, 0, 8);

	queued_retval = ERROR_OK;
	art_swdio_flush();
	

	for (i = 0; i < transaction_queue_length; i++) {
		//if ((i % 10) == 0) LOG_INFO("run %d", i);
		if (transaction_queue[i].read) {
			response_swdio_read_reg(&ack, &data, &parity);
			if (transaction_queue[i].dst != NULL)
				*transaction_queue[i].dst = data;
			if (ack != SWD_ACK_OK) {
				//LOG_INFO("!!!ack != SWD_ACK_OK1");
				queued_retval = swd_ack_to_error_code(ack);
				goto skip;
			}
		} else {
			response_swdio_write_reg(&ack);
			if (ack == SWD_ACK_WAIT) {
			    LOG_INFO("Wait");
			    swd_clear_sticky_errors();
			}
			
			
			if (ack != SWD_ACK_OK) {
				//LOG_INFO("!!!ack != SWD_ACK_OK2");
				queued_retval = swd_ack_to_error_code(ack);
				goto skip;
			}
		}
	}

skip:
	//LOG_INFO("ending");
	transaction_queue_length = 0;

	int retval = queued_retval;
	queued_retval = ERROR_OK;
	//LOG_INFO("SWD queue return value: %02x", retval);
	art_flush_queues();
	return retval;
}

void queue_swdio_write_reg(uint8_t cmd, uint32_t data);
void queue_swdio_read_reg(uint8_t cmd);

static void art_queue_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk) {
	transaction_queue[transaction_queue_length].read = 1;
	transaction_queue[transaction_queue_length].dst = value;
	transaction_queue[transaction_queue_length].cmd = cmd;
	transaction_queue_length++;
	queue_swdio_read_reg(cmd);
}

static void art_queue_swd_write_reg(uint8_t cmd, uint32_t data, uint32_t ap_delay_clk) {
	transaction_queue[transaction_queue_length].read = 0;
	transaction_queue[transaction_queue_length].dst = NULL;
	transaction_queue[transaction_queue_length].cmd = cmd;
	transaction_queue_length++;
	queue_swdio_write_reg(cmd, data);
}

const struct swd_driver art_bitbang_swd = {
	.init = bitbang_swd_init,
	.switch_seq = bitbang_swd_switch_seq,
	.read_reg = art_queue_swd_read_reg,
	.write_reg = art_queue_swd_write_reg,
	.run = bitbang_swd_run_queue,
};
