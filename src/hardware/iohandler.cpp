/*
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *
 *  Copyright (C) 2020-2021  The DOSBox Staging Team
 *  Copyright (C) 2002-2021  The DOSBox Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 */

#include "inout.h"

#include <cassert>
#include <limits>
#include <cstring>
#include <unordered_map>

#include "setup.h"
#include "cpu.h"
#include "../src/cpu/lazyflags.h"
#include "callback.h"

//#define ENABLE_PORTLOG

constexpr int io_widths = 3; // byte, word, and dword

static std::unordered_map<uint16_t, io_read_f> io_read_handlers[io_widths] = {};
constexpr auto &io_read_byte_handler = io_read_handlers[0];
constexpr auto &io_read_word_handler = io_read_handlers[1];
constexpr auto &io_read_dword_handler = io_read_handlers[2];

static std::unordered_map<uint16_t, io_write_f> io_write_handlers[io_widths] = {};
constexpr auto &io_write_byte_handler = io_write_handlers[0];
constexpr auto &io_write_word_handler = io_write_handlers[1];
constexpr auto &io_write_dword_handler = io_write_handlers[2];

static uint8_t no_read(io_port_t port)
{
	LOG(LOG_IO, LOG_WARN)
	("IOBUS: Unexpected read from %04xh; blocking", port);
	return 0xff;
}

static uint8_t read_byte_from_port(io_port_t port)
{
	const auto reader = io_read_byte_handler.find(port);
	const auto value = reader != io_read_byte_handler.end()
	                           ? reader->second(port,  io_width_t::byte)
	                           : no_read(port);
	assert(value <= UINT8_MAX);
	return static_cast<uint8_t>(value);
}

static uint16_t read_word_from_port(io_port_t port)
{
	const auto reader = io_read_word_handler.find(port);
	const auto value = reader != io_read_word_handler.end()
	                           ? reader->second(port, io_width_t::word)
	                           : static_cast<io_val_t>(
	                                     read_byte_from_port(port) |
	                                     (read_byte_from_port(port + 1) << 8));
	assert(value <= UINT16_MAX);
	return static_cast<uint16_t>(value);
}

static uint32_t read_dword_from_port(io_port_t port)
{
	const auto reader = io_read_dword_handler.find(port);
	const auto value = reader != io_read_dword_handler.end()
	                           ? reader->second(port,  io_width_t::dword)
	                           : static_cast<io_val_t>(
	                                     read_word_from_port(port) |
	                                     (read_word_from_port(port + 2) << 16));
	assert(value <= UINT32_MAX);
	return value;
}

static void no_write(io_port_t port, uint8_t val)
{
	LOG(LOG_IO, LOG_WARN)("IOBUS: Unexpected write of %u to %04xh; blocking", val, port);
}

static void write_byte_to_port(io_port_t port, uint8_t val)
{
	const auto writer = io_write_byte_handler.find(port);
	if (writer != io_write_byte_handler.end())
		writer->second(port, val, io_width_t::byte);
	else
		no_write(port, val);
}

static void write_word_to_port(io_port_t port, uint16_t val)
{
	const auto writer = io_write_word_handler.find(port);
	if (writer != io_write_word_handler.end()) {
		writer->second(port, val, io_width_t::word);
	}
	else {
		write_byte_to_port(port, val & 0xff);
		write_byte_to_port(port + 1, val >> 8);
	}
}

static void write_dword_to_port(io_port_t port, uint32_t val)
{
	const auto writer = io_write_dword_handler.find(port);
	if (writer != io_write_dword_handler.end()) {
		writer->second(port, val, io_width_t::dword);
	}
	else {
		write_word_to_port(port, val & 0xffff);
		write_word_to_port(port + 2, val >> 16);
	}
}

void IO_RegisterReadHandler(io_port_t port, io_read_f handler, io_width_t max_width, io_port_t range)
{
	while (range--) {
		io_read_byte_handler[port] = handler;
		if (max_width == io_width_t::word || max_width == io_width_t::dword)
			io_read_word_handler[port] = handler;
		if (max_width == io_width_t::dword)
			io_read_dword_handler[port] = handler;
		++port;
	}
}

void IO_RegisterWriteHandler(io_port_t port, io_write_f handler, io_width_t max_width, io_port_t range)
{
	while (range--) {
		io_write_byte_handler[port] = handler;
		if (max_width == io_width_t::word || max_width == io_width_t::dword)
			io_write_word_handler[port] = handler;
		if (max_width == io_width_t::dword)
			io_write_dword_handler[port] = handler;
		++port;
	}
}

void IO_FreeReadHandler(io_port_t port, io_width_t max_width, io_port_t range)
{
	while (range--) {
		io_read_byte_handler.erase(port);
		if (max_width == io_width_t::word || max_width == io_width_t::dword)
			io_read_word_handler.erase(port);
		if (max_width == io_width_t::dword)
			io_read_dword_handler.erase(port);
		port++;
	}
}

void IO_FreeWriteHandler(io_port_t port, io_width_t width, io_port_t range)
{
	while (range--) {
		io_write_byte_handler.erase(port);
		if (width == io_width_t::word || width == io_width_t::dword)
			io_write_word_handler.erase(port);
		if (width == io_width_t::dword)
			io_write_dword_handler.erase(port);
		++port;
	}
}

void IO_ReadHandleObject::Install(io_port_t port, io_read_f handler, io_width_t max_width, io_port_t range)
{
	if(!installed) {
		installed=true;
		m_port=port;
		m_width = max_width;
		m_range=range;
		IO_RegisterReadHandler(port, handler, max_width, range);
	} else
		E_Exit("io_read_f already installed port %u", port);
}

void IO_WriteHandleObject::Install(io_port_t port, io_write_f handler, io_width_t max_width, io_port_t range)
{
	if(!installed) {
		installed=true;
		m_port=port;
		m_width = max_width;
		m_range=range;
		IO_RegisterWriteHandler(port, handler, max_width, range);
	} else
		E_Exit("io_write_f already installed port %u", port);
}

void IO_ReadHandleObject::Uninstall()
{
	if (!installed)
		return;
	IO_FreeReadHandler(m_port, m_width, m_range);
	installed = false;
}

IO_ReadHandleObject::~IO_ReadHandleObject()
{
	Uninstall();
}

void IO_WriteHandleObject::Uninstall()
{
	if (!installed)
		return;
	IO_FreeWriteHandler(m_port, m_width, m_range);
	installed = false;
}

IO_WriteHandleObject::~IO_WriteHandleObject()
{
	Uninstall();
	// LOG_MSG("IOBUS: FreeWritehandler called with port %04x",
	// static_cast<uint32_t>(m_port));
}

struct IOF_Entry {
	Bitu cs;
	Bitu eip;
};

#define IOF_QUEUESIZE 16
static struct {
	Bitu used;
	IOF_Entry entries[IOF_QUEUESIZE];
} iof_queue;

static Bits IOFaultCore(void) {
	CPU_CycleLeft+=CPU_Cycles;
	CPU_Cycles=1;
	Bits ret=CPU_Core_Full_Run();
	CPU_CycleLeft+=CPU_Cycles;
	if (ret<0) E_Exit("Got a dosbox close machine in IO-fault core?");
	if (ret)
		return ret;
	if (!iof_queue.used) E_Exit("IO-faul Core without IO-faul");
	IOF_Entry * entry=&iof_queue.entries[iof_queue.used-1];
	if (entry->cs == SegValue(cs) && entry->eip==reg_eip)
		return -1;
	return 0;
}


/* Some code to make io operations take some virtual time. Helps certain
 * games with their timing of certain operations
 */

constexpr double IODELAY_READ_MICROS = 1.0;
constexpr double IODELAY_WRITE_MICROS = 0.75;

constexpr int32_t IODELAY_READ_MICROSk = static_cast<int32_t>(
        1024 / IODELAY_READ_MICROS);
constexpr int32_t IODELAY_WRITE_MICROSk = static_cast<int32_t>(
        1024 / IODELAY_WRITE_MICROS);

inline void IO_USEC_read_delay() {
	Bits delaycyc = CPU_CycleMax/IODELAY_READ_MICROSk;
	if(GCC_UNLIKELY(delaycyc > CPU_Cycles)) delaycyc = CPU_Cycles;
	CPU_Cycles -= delaycyc;
	CPU_IODelayRemoved += delaycyc;
}

inline void IO_USEC_write_delay() {
	Bits delaycyc = CPU_CycleMax/IODELAY_WRITE_MICROSk;
	if(GCC_UNLIKELY(delaycyc > CPU_Cycles)) delaycyc = CPU_Cycles;
	CPU_Cycles -= delaycyc;
	CPU_IODelayRemoved += delaycyc;
}

#ifdef ENABLE_PORTLOG
static Bit8u crtc_index = 0;

void log_io(io_width_t width, bool write, io_port_t port, io_val_t val)
{
	switch(width) {
	case io_width_t::byte: val &= 0xff; break;
	case io_width_t::word: val &= 0xffff; break;
	}
	if (write) {
		// skip the video cursor position spam
		if (port==0x3d4) {
			if (width==io_width_t::byte) crtc_index = (Bit8u)val;
			else if(width==io_width_t::word) crtc_index = (Bit8u)(val>>8);
		}
		if (crtc_index==0xe || crtc_index==0xf) {
			if((width==io_width_t::byte && (port==0x3d4 || port==0x3d5))||(width==io_width_t::word && port==0x3d4))
				return;
		}

		switch(port) {
		//case 0x020: // interrupt command
		//case 0x040: // timer 0
		//case 0x042: // timer 2
		//case 0x043: // timer control
		//case 0x061: // speaker control
		case 0x3c8: // VGA palette
		case 0x3c9: // VGA palette
		// case 0x3d4: // VGA crtc
		// case 0x3d5: // VGA crtc
		// case 0x3c4: // VGA seq
		// case 0x3c5: // VGA seq
			break;
		default:
			LOG_MSG("IOSBUS: write width=%u bytes, % 4x % 4x, cs:ip %04x:%04x",
			        static_cast<uint8_t>(width), port, val, SegValue(cs), reg_eip);
			break;
		}
	} else {
		switch(port) {
		//case 0x021: // interrupt status
		//case 0x040: // timer 0
		//case 0x042: // timer 2
		//case 0x061: // speaker control
		case 0x201: // joystick status
		case 0x3c9: // VGA palette
		// case 0x3d4: // VGA crtc index
		// case 0x3d5: // VGA crtc
		case 0x3da: // display status - a real spammer
			// don't log for the above cases
			break;
		default:
			LOG_MSG("IOBUS: read width=%u bytes % 4x % 4x,\t\tcs:ip %04x:%04x",
			        static_cast<uint8_t>(width), port, val, SegValue(cs), reg_eip);
			break;
		}
	}
}
#else
#define log_io(W, X, Y, Z)
#endif

void IO_WriteB(io_port_t port, uint8_t val)
{
	log_io(io_width_t::byte, true, port, val);
	if (GCC_UNLIKELY(GETFLAG(VM) && (CPU_IO_Exception(port,1)))) {
		LazyFlags old_lflags;
		memcpy(&old_lflags,&lflags,sizeof(LazyFlags));
		CPU_Decoder * old_cpudecoder;
		old_cpudecoder=cpudecoder;
		cpudecoder=&IOFaultCore;
		IOF_Entry * entry=&iof_queue.entries[iof_queue.used++];
		entry->cs=SegValue(cs);
		entry->eip=reg_eip;
		CPU_Push16(SegValue(cs));
		CPU_Push16(reg_ip);
		Bit8u old_al = reg_al;
		Bit16u old_dx = reg_dx;
		reg_al = val;
		reg_dx = port;
		RealPt icb = CALLBACK_RealPointer(call_priv_io);
		SegSet16(cs,RealSeg(icb));
		reg_eip = RealOff(icb)+0x08;
		CPU_Exception(cpu.exception.which,cpu.exception.error);

		DOSBOX_RunMachine();
		iof_queue.used--;

		reg_al = old_al;
		reg_dx = old_dx;
		memcpy(&lflags,&old_lflags,sizeof(LazyFlags));
		cpudecoder=old_cpudecoder;
	}
	else {
		IO_USEC_write_delay();
		write_byte_to_port(port, val);
	}
}

void IO_WriteW(io_port_t port, uint16_t val)
{
	log_io(io_width_t::word, true, port, val);
	if (GCC_UNLIKELY(GETFLAG(VM) && (CPU_IO_Exception(port,2)))) {
		LazyFlags old_lflags;
		memcpy(&old_lflags,&lflags,sizeof(LazyFlags));
		CPU_Decoder * old_cpudecoder;
		old_cpudecoder=cpudecoder;
		cpudecoder=&IOFaultCore;
		IOF_Entry * entry=&iof_queue.entries[iof_queue.used++];
		entry->cs=SegValue(cs);
		entry->eip=reg_eip;
		CPU_Push16(SegValue(cs));
		CPU_Push16(reg_ip);
		Bit16u old_ax = reg_ax;
		Bit16u old_dx = reg_dx;
		reg_ax = val;
		reg_dx = port;
		RealPt icb = CALLBACK_RealPointer(call_priv_io);
		SegSet16(cs,RealSeg(icb));
		reg_eip = RealOff(icb)+0x0a;
		CPU_Exception(cpu.exception.which,cpu.exception.error);

		DOSBOX_RunMachine();
		iof_queue.used--;

		reg_ax = old_ax;
		reg_dx = old_dx;
		memcpy(&lflags,&old_lflags,sizeof(LazyFlags));
		cpudecoder=old_cpudecoder;
	}
	else {
		IO_USEC_write_delay();
		write_word_to_port(port, val);
	}
}

void IO_WriteD(io_port_t port, uint32_t val)
{
	log_io(io_width_t::dword, true, port, val);
	if (GCC_UNLIKELY(GETFLAG(VM) && (CPU_IO_Exception(port,4)))) {
		LazyFlags old_lflags;
		memcpy(&old_lflags,&lflags,sizeof(LazyFlags));
		CPU_Decoder * old_cpudecoder;
		old_cpudecoder=cpudecoder;
		cpudecoder=&IOFaultCore;
		IOF_Entry * entry=&iof_queue.entries[iof_queue.used++];
		entry->cs=SegValue(cs);
		entry->eip=reg_eip;
		CPU_Push16(SegValue(cs));
		CPU_Push16(reg_ip);
		Bit32u old_eax = reg_eax;
		Bit16u old_dx = reg_dx;
		reg_eax = val;
		reg_dx = port;
		RealPt icb = CALLBACK_RealPointer(call_priv_io);
		SegSet16(cs,RealSeg(icb));
		reg_eip = RealOff(icb)+0x0c;
		CPU_Exception(cpu.exception.which,cpu.exception.error);

		DOSBOX_RunMachine();
		iof_queue.used--;

		reg_eax = old_eax;
		reg_dx = old_dx;
		memcpy(&lflags,&old_lflags,sizeof(LazyFlags));
		cpudecoder=old_cpudecoder;
	} else {
		write_dword_to_port(port, val);
	}
}

uint8_t IO_ReadB(io_port_t port)
{
	uint8_t retval;
	if (GCC_UNLIKELY(GETFLAG(VM) && (CPU_IO_Exception(port,1)))) {
		LazyFlags old_lflags;
		memcpy(&old_lflags,&lflags,sizeof(LazyFlags));
		CPU_Decoder * old_cpudecoder;
		old_cpudecoder=cpudecoder;
		cpudecoder=&IOFaultCore;
		IOF_Entry * entry=&iof_queue.entries[iof_queue.used++];
		entry->cs=SegValue(cs);
		entry->eip=reg_eip;
		CPU_Push16(SegValue(cs));
		CPU_Push16(reg_ip);
		Bit8u old_al = reg_al;
		Bit16u old_dx = reg_dx;
		reg_dx = port;
		RealPt icb = CALLBACK_RealPointer(call_priv_io);
		SegSet16(cs,RealSeg(icb));
		reg_eip = RealOff(icb)+0x00;
		CPU_Exception(cpu.exception.which,cpu.exception.error);

		DOSBOX_RunMachine();
		iof_queue.used--;

		retval = reg_al;
		reg_al = old_al;
		reg_dx = old_dx;
		memcpy(&lflags,&old_lflags,sizeof(LazyFlags));
		cpudecoder=old_cpudecoder;
		return retval;
	}
	else {
		IO_USEC_read_delay();
		retval = read_byte_from_port(port);
	}
	log_io(io_width_t::byte, false, port, retval);
	return retval;
}

uint16_t IO_ReadW(io_port_t port)
{
	uint16_t retval;
	if (GCC_UNLIKELY(GETFLAG(VM) && (CPU_IO_Exception(port,2)))) {
		LazyFlags old_lflags;
		memcpy(&old_lflags,&lflags,sizeof(LazyFlags));
		CPU_Decoder * old_cpudecoder;
		old_cpudecoder=cpudecoder;
		cpudecoder=&IOFaultCore;
		IOF_Entry * entry=&iof_queue.entries[iof_queue.used++];
		entry->cs=SegValue(cs);
		entry->eip=reg_eip;
		CPU_Push16(SegValue(cs));
		CPU_Push16(reg_ip);
		Bit16u old_ax = reg_ax;
		Bit16u old_dx = reg_dx;
		reg_dx = port;
		RealPt icb = CALLBACK_RealPointer(call_priv_io);
		SegSet16(cs,RealSeg(icb));
		reg_eip = RealOff(icb)+0x02;
		CPU_Exception(cpu.exception.which,cpu.exception.error);

		DOSBOX_RunMachine();
		iof_queue.used--;

		retval = reg_ax;
		reg_ax = old_ax;
		reg_dx = old_dx;
		memcpy(&lflags,&old_lflags,sizeof(LazyFlags));
		cpudecoder=old_cpudecoder;
	}
	else {
		IO_USEC_read_delay();
		retval = read_word_from_port(port);
	}
	log_io(io_width_t::word, false, port, retval);
	return retval;
}

uint32_t IO_ReadD(io_port_t port)
{
	uint32_t retval;
	if (GCC_UNLIKELY(GETFLAG(VM) && (CPU_IO_Exception(port,4)))) {
		LazyFlags old_lflags;
		memcpy(&old_lflags,&lflags,sizeof(LazyFlags));
		CPU_Decoder * old_cpudecoder;
		old_cpudecoder=cpudecoder;
		cpudecoder=&IOFaultCore;
		IOF_Entry * entry=&iof_queue.entries[iof_queue.used++];
		entry->cs=SegValue(cs);
		entry->eip=reg_eip;
		CPU_Push16(SegValue(cs));
		CPU_Push16(reg_ip);
		Bit32u old_eax = reg_eax;
		Bit16u old_dx = reg_dx;
		reg_dx = port;
		RealPt icb = CALLBACK_RealPointer(call_priv_io);
		SegSet16(cs,RealSeg(icb));
		reg_eip = RealOff(icb)+0x04;
		CPU_Exception(cpu.exception.which,cpu.exception.error);

		DOSBOX_RunMachine();
		iof_queue.used--;

		retval = reg_eax;
		reg_eax = old_eax;
		reg_dx = old_dx;
		memcpy(&lflags,&old_lflags,sizeof(LazyFlags));
		cpudecoder=old_cpudecoder;
	} else {
		retval = read_dword_from_port(port);
	}

	log_io(io_width_t::dword, false, port, retval);
	return retval;
}

class IO final : public Module_base {
public:
	IO(Section* configuration):Module_base(configuration){
		iof_queue.used = 0;
	}
	~IO()
	{
		size_t total_bytes = 0u;
		for (uint8_t i = 0; i < io_widths; ++i) {
			const size_t readers = io_read_handlers[i].size();
			const size_t writers = io_write_handlers[i].size();
			DEBUG_LOG_MSG("IOBUS: Releasing %lu read and %lu write %d-bit port handlers", readers,
			              writers, 8 << i);
			total_bytes += readers * sizeof(io_read_f) + sizeof(io_read_handlers[i]);
			total_bytes += writers * sizeof(io_write_f) + sizeof(io_write_handlers[i]);
			io_read_handlers[i].clear();
			io_write_handlers[i].clear();
		}
		DEBUG_LOG_MSG("IOBUS: Handlers consumed %lu total bytes", total_bytes);
	}
};

static IO* test;

void IO_Destroy(Section*) {
	delete test;
}

void IO_Init(Section * sect) {
	test = new IO(sect);
	sect->AddDestroyFunction(&IO_Destroy);
}
