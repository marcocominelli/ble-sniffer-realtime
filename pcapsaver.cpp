/*
 * Copyright 2020 Marco Cominelli
 * Copyright 2018 Francesco Gringoli
 *
 * This is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this. If not, see <https://www.gnu.org/licenses/>.
 */

#include <pcap.h>
#include <string.h>
#include <assert.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <stdint.h>
#include <math.h>

#include "pcapsaver.h"

struct timeval *packet::getts(void)
{
	return &pcaphdr.ts;
}


bool operator<(const packet &p1, const packet &p2)
{
	assert(p1.hasheader);
	assert(p2.hasheader);
	double t1 = p1.pcaphdr.ts.tv_sec + double(p1.pcaphdr.ts.tv_usec) / 1000000.0;
	double t2 = p2.pcaphdr.ts.tv_sec + double(p2.pcaphdr.ts.tv_usec) / 1000000.0;

	if(t1 < t2) {
		return true;
	}

	return false;
}


void packet::setdelay(int us)
{
	assert(hasheader);
	double tm = pcaphdr.ts.tv_sec + double(pcaphdr.ts.tv_usec + us) / 1000000.0;
	if(tm > 0) {
		pcaphdr.ts.tv_sec = floor(tm);
		pcaphdr.ts.tv_usec = (tm - pcaphdr.ts.tv_sec) * 1000000.0;
	} else {
		pcaphdr.ts.tv_sec = ceil(tm);
		pcaphdr.ts.tv_usec = (tm + pcaphdr.ts.tv_sec) * 1000000.0;
	}
}


struct pcap_pkthdr *packet::getpcaphdr(void)
{
	assert(hasheader);
	return &pcaphdr;
}


uint8_t *packet::getdata(void)
{
	assert(haspayload);
	return blob;
}


void packet::setnext(packet *p)
{
	next = p;
}


packet *packet::getnext()
{
	return next;
}


void packet::setheader(int channel, uint32_t aa, struct timeval *ts, uint32_t ptype)
{
	assert(channel >=0 && channel <= 39);

	if(channel <= 10) {
		channel += 1;
	} else if(channel <= 36) {
		channel += 2;
	} else if(channel == 37) {
		channel = 0;
	} else if(channel == 38) {
		channel = 12;
	}
	memset(blob, 0, sizeof(struct ble_hdr));
	struct ble_hdr *bh = (struct ble_hdr *) blob;

	if((ptype & ~BLE_MODULATION_MASK) != 0) {
		printf("ptype = %u\nmask = %u\n", ptype, ~BLE_MODULATION_MASK);
	}
	assert((ptype & ~BLE_MODULATION_MASK) == 0);
	bh->flags = BLE_DEWHITENED | CRC_CHECKED | CRC_VALID | ptype; 
	bh->channel = channel;
	uint8_t *_aa = blob + sizeof(struct ble_hdr);
	_aa[0] = (aa >> 0) & 0xff;
	_aa[1] = (aa >> 8) & 0xff;
	_aa[2] = (aa >> 16) & 0xff;
	_aa[3] = (aa >> 24) & 0xff;
	pcaphdr.ts = *ts;
	pcaphdr.ts.tv_sec += 1;
	hasheader = true;
}


void packet::setpayload(int length, uint8_t *pl)
{
	assert(length <= MAX_PAYLOAD);
	memcpy(blob + sizeof(struct ble_hdr) + AA_LENGTH, pl, length);
	pcaphdr.len = sizeof(struct ble_hdr) + AA_LENGTH + length;
	pcaphdr.caplen = sizeof(struct ble_hdr) + AA_LENGTH + length;
	haspayload = true;
}


int pcapmerger::create(const char *filename)
{
	dev = pcap_open_dead(DLT_BLUETOOTH_LE_LL_WITH_PHDR, 255);
	if(!dev) {
		fprintf(stderr, "Cannot open dead device\n");
		return -1;
	}
	capfile = pcap_dump_open(dev, filename);
	if(!capfile) {
		fprintf(stderr, "Cannot open file for writing\n");
		return -1;
	}

	return 0;
}

int pcapmerger::addpacket(struct timeval *ts, int channel,
			   uint32_t aa, int length, uint8_t *payload, int ptype)
{
	packet *newpacket = new packet;
	if(!newpacket) {
		fprintf(stderr, "Cannot push new packet\n");
		return -1;
	}
	newpacket->setheader(channel, aa, ts, ptype);
	newpacket->setpayload(length, payload);

	if(head1 == NULL) {
		head1 = tail1 = newpacket;
	}
	else {
		tail1->setnext(newpacket);
		tail1 = newpacket;
	}
}

/*
int pcapmerger::addpacket2(struct timeval *ts, int channel,
                           uint32_t aa, int length, uint8_t *payload, int ptype)
{
        packet *newpacket = new packet;
        if(!newpacket) {
                fprintf(stderr, "Cannot push new packet\n");
                return -1;
        }
        newpacket->setheader(channel, aa, ts, ptype);
        newpacket->setpayload(length, payload);

        if(head2 == NULL) {
                head2 = tail2 = newpacket;
        }
        else {
                tail2->setnext(newpacket);
                tail2 = newpacket;
        }
}
*/

void pcapmerger::flush(void)
{
	packet *p1 = head1;
	packet *p2 = head2;

	while(p1 && p2) {
		if(*p1 < *p2) {
			// packet *p = p1;
			// printf("p1 %d.%06d\n", (int) p->getts()->tv_sec, (int) p->getts()->tv_usec);
			pcap_dump((u_char *) capfile, p1->getpcaphdr(), p1->getdata());
			packet *next = p1->getnext();
			delete(p1);
			p1 = next;
		}
		else {
			// packet *p = p2;
			// printf("p2 %d.%06d\n", (int) p->getts()->tv_sec, (int) p->getts()->tv_usec);
			pcap_dump((u_char *) capfile, p2->getpcaphdr(), p2->getdata());
			packet *next = p2->getnext();
			delete(p2);
			p2 = next;
		}
		// printf("\n");
	}

	while(p1) {
		// packet *p = p1;
		// printf("p1 %d.%06d\n", (int) p->getts()->tv_sec, (int) p->getts()->tv_usec);
		pcap_dump((u_char *) capfile, p1->getpcaphdr(), p1->getdata());
		packet *next = p1->getnext();
		delete(p1);
		p1 = next;
	}

	while(p2) {
		// packet *p = p2;
		// printf("p2 %d.%06d\n", (int) p->getts()->tv_sec, (int) p->getts()->tv_usec);
		pcap_dump((u_char *) capfile, p2->getpcaphdr(), p2->getdata());
		packet *next = p2->getnext();
		delete(p2);
		p2 = next;
	}
}

void pcapmerger::finalise(void)
{
	pcap_dump_close(capfile);
}

