/*
 *  SPDX-License-Identifier: GPL-2.0-or-later
 *
 *  Copyright (C) 2021-2021  The DOSBox Staging Team
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

#include <gtest/gtest.h>

#include <cstdint>
#include <algorithm>

namespace {

constexpr auto iterations = 10000;

static io_val_t mem = 0;

// Writes the input value as-is to memory
static uint8_t write_byte(io_port_t port, uint8_t val, io_width_t){
	mem = val;
}

// Writes the input value as-is to memory
static uint16_t write_word(io_port_t port, uint16_t val, io_width_t){
	mem = val;
}

// Writes the input value as-is to memory
static uint32_t write_dword(io_port_t port, uint32_t val, io_width_t){
	mem = val;
}

// Returns the mem  value cast to the 8-bit range
static uint8_t read_byte(io_port_t port, io_width_t){
	return static_cast<uint8_t>(mem);
}

// Returns the memory value as-is
static uint16_t read_word(io_port_t port, io_width_t){
	return static_cast<uint16_t>(mem);
}

// Returns the memory value scaled up to the 32-bit range
static uint32_t read_dword(io_port_t port, io_width_t){
	return mem;
}

TEST(IOHandler, ReadBytes)
{
	io_port_t port = 1;
	IO_RegisterReadHandler(port, read_byte, io_width_t::byte);
	IO_RegisterWriteHandler(port, write_byte, io_width_t::byte);

	uint8_t val = 0;
	IO_WriteB(port, val);
	EXPECT_EQ(IO_ReadB(port), val);

	uint8_t val = UINT8_MAX;
	IO_WriteB(port, val);
	EXPECT_EQ(IO_ReadB(port), val);


	RWQueue<int> q(65);
	for (int iteration = 0; iteration != 128;
	     ++iteration) { // check there's no problem with mismatch
		            // between nominal and allocated capacity
		EXPECT_EQ(q.MaxCapacity(), 65);
		EXPECT_EQ(q.Size(), 0);
		EXPECT_TRUE(q.IsEmpty());
		q.Enqueue(0);
		EXPECT_EQ(q.MaxCapacity(), 65);
		EXPECT_EQ(q.Size(), 1);
		EXPECT_FALSE(q.IsEmpty());
		for (int i = 1; i != 65; ++i)
			q.Enqueue(std::move(i));
		EXPECT_EQ(q.Size(), 65);
		EXPECT_FALSE(q.IsEmpty());

		// Basic dequeue
		int item;
		item = q.Dequeue();
		EXPECT_EQ(item, 0);
		for (int i = 1; i != 65; ++i) {
			item = q.Dequeue();
			EXPECT_EQ(item, i);
		}
		EXPECT_EQ(item, 64);
		EXPECT_TRUE(q.IsEmpty());
	}
}

TEST(RWQueue, TrivialZeroCapacity)
{
	// Zero capacity
	EXPECT_DEBUG_DEATH({ RWQueue<int> q(0); }, "");
}

void rw_consume_trivial(RWQueue<int> *q, const size_t *max_depth)
{
	int item;
	for (int i = 0; i != iterations; ++i) {
		EXPECT_TRUE(q->Size() <= *max_depth);
		item = q->Dequeue();
		EXPECT_EQ(item, i);
	}
}

void rw_produce_copy_trivial(RWQueue<int> *q, const size_t *max_depth)
{
	for (int i = 0; i != iterations; ++i) {
		q->Enqueue(i);
		EXPECT_TRUE(q->Size() <= *max_depth);
	}
}

void rw_produce_move_trivial(RWQueue<int> *q, const size_t *max_depth)
{
	for (int i = 0; i != iterations; ++i) {
		q->Enqueue(std::move(i));
		EXPECT_TRUE(q->Size() <= *max_depth);
	}
}

TEST(RWQueue, TrivialCopyAsync)
{
	const size_t max_depth = 8;
	RWQueue<int> q(max_depth);

	std::thread writer(rw_produce_copy_trivial, &q, &max_depth);
	std::thread reader(rw_consume_trivial, &q, &max_depth);

	writer.join();
	reader.join();

	// Make sure we've consumed all produced items and the queue is empty
	EXPECT_EQ(q.Size(), 0);
}

TEST(RWQueue, TrivialMoveAsync)
{
	const size_t max_depth = 8;
	RWQueue<int> q(max_depth);

	std::thread writer(rw_produce_move_trivial, &q, &max_depth);
	std::thread reader(rw_consume_trivial, &q, &max_depth);

	writer.join();
	reader.join();

	// Make sure we've consumed all produced items and the queue is empty
	EXPECT_EQ(q.Size(), 0);
}


using container_t = std::vector<int16_t>;

TEST(RWQueue,ContainerSerial)
{
	RWQueue<container_t> q(65);
	for (int iteration = 0; iteration != 128;
	     ++iteration) { // check there's no problem with mismatch
		            // between nominal and allocated capacity
		EXPECT_EQ(q.MaxCapacity(), 65);
		EXPECT_EQ(q.Size(), 0);
		EXPECT_TRUE(q.IsEmpty());

		container_t v(iteration + 1); 
		v[iteration] = iteration;
		q.Enqueue(v);
		EXPECT_EQ(v.size(), iteration + 1); // check copy

		EXPECT_EQ(q.MaxCapacity(), 65);
		EXPECT_EQ(q.Size(), 1);
		EXPECT_FALSE(q.IsEmpty());
		for (int i = 1; i != 65; ++i) {
			container_t v(i + 1); 
			v[i] = i;
			q.Enqueue(std::move(v));
			EXPECT_EQ(v.size(), 0); // check move
		}
		EXPECT_EQ(q.Size(), 65);
		EXPECT_FALSE(q.IsEmpty());

		// Basic dequeue
		v = q.Dequeue();
		EXPECT_EQ(v[0], 0);
		EXPECT_EQ(v.size(), iteration + 1);

		for (int i = 1; i != 65; ++i) {
			v = q.Dequeue();
			EXPECT_EQ(v[i], i);
			EXPECT_EQ(v.size(), i + 1);
		}
		EXPECT_EQ(v[64], 64);
		EXPECT_EQ(v.size(), 65);
		EXPECT_TRUE(q.IsEmpty());
	}
}

TEST(RWQueue,ContainerZeroCapacity)
{
	// Zero capacity
	EXPECT_DEBUG_DEATH({ RWQueue<container_t> q(0); }, "");
}

void rw_consume_container(RWQueue<container_t> *q, const size_t *max_depth)
{
	container_t v;
	for (int i = 0; i != iterations; ++i) {
		EXPECT_TRUE(q->Size() <= *max_depth);
		v = q->Dequeue();
		EXPECT_EQ(v[i], i);
		EXPECT_EQ(v.size(), i + 1);
	}
}

void rw_produce_copy_container(RWQueue<container_t> *q, const size_t *max_depth)
{
	for (int i = 0; i != iterations; ++i) {
		container_t v(i + 1);
		v[i] = i;
		q->Enqueue(v);
		EXPECT_EQ(v.size(), i + 1); // check copy
		EXPECT_TRUE(q->Size() <= *max_depth);
	}
}

void rw_produce_move_container(RWQueue<container_t> *q, const size_t *max_depth)
{
	for (int i = 0; i != iterations; ++i) {
		container_t v(i + 1);
		v[i] = i;
		q->Enqueue(std::move(v));
		EXPECT_EQ(v.size(), 0); // check move
		EXPECT_TRUE(q->Size() <= *max_depth);
	}
}

TEST(RWQueue,ContainerCopyAsync)
{
	const size_t max_depth = 8;
	RWQueue<container_t> q(max_depth);

	std::thread writer(rw_produce_copy_container, &q, &max_depth);
	std::thread reader(rw_consume_container, &q, &max_depth);

	writer.join();
	reader.join();

	// Make sure we've consumed all produced items and the queue is empty
	EXPECT_EQ(q.Size(), 0);
}

TEST(RWQueue,ContainerMoveAsync)
{
	const size_t max_depth = 8;
	RWQueue<container_t> q(max_depth);

	std::thread writer(rw_produce_move_container, &q, &max_depth);
	std::thread reader(rw_consume_container, &q, &max_depth);

	writer.join();
	reader.join();

	// Make sure we've consumed all produced items and the queue is empty
	EXPECT_EQ(q.Size(), 0);
}

} // namespace
