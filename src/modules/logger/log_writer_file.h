/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <px4_defines.h>
#include <stdint.h>
#include <pthread.h>
#include <drivers/drv_hrt.h>
#include <perf/perf_counter.h>

namespace px4
{
namespace logger
{

/**
 * @class LogWriterFile
 * Writes logging data to a file
 */
class LogWriterFile
{
public:
	_EXT_ITCM LogWriterFile(size_t buffer_size);
	_EXT_ITCM ~LogWriterFile();

	_EXT_ITCM bool init();

	/**
	 * start the thread
	 * @return 0 on success, error number otherwise (@see pthread_create)
	 */
	_EXT_ITCM int thread_start();

	_EXT_ITCM void thread_stop();

	_EXT_ITCM void start_log(const char *filename);

	_EXT_ITCM void stop_log();

	_EXT_ITCM bool is_started() const { return _should_run; }

	/** @see LogWriter::write_message() */
	_EXT_ITCM int write_message(void *ptr, size_t size, uint64_t dropout_start = 0);

	_EXT_ITCM void lock()
	{
		pthread_mutex_lock(&_mtx);
	}

	_EXT_ITCM void unlock()
	{
		pthread_mutex_unlock(&_mtx);
	}

	_EXT_ITCM void notify()
	{
		pthread_cond_broadcast(&_cv);
	}

	_EXT_ITCM size_t get_total_written() const
	{
		return _total_written;
	}

	_EXT_ITCM size_t get_buffer_size() const
	{
		return _buffer_size;
	}

	_EXT_ITCM size_t get_buffer_fill_count() const
	{
		return _count;
	}

	_EXT_ITCM void set_need_reliable_transfer(bool need_reliable)
	{
		_need_reliable_transfer = need_reliable;
	}

	_EXT_ITCM bool need_reliable_transfer() const
	{
		return _need_reliable_transfer;
	}

	_EXT_ITCM pthread_t thread_id() const { return _thread; }

private:
	_EXT_ITCM static void *run_helper(void *);

	_EXT_ITCM void run();

	_EXT_ITCM size_t get_read_ptr(void **ptr, bool *is_part);

	_EXT_ITCM void mark_read(size_t n)
	{
		_count -= n;
	}

	/**
	 * permanently store the ulog file name for the hardfault crash handler, so that it can
	 * append crash logs to the last ulog file.
	 * @param log_file path to the log file
	 * @return 0 on success, <0 errno otherwise
	 */
	_EXT_ITCM int hardfault_store_filename(const char *log_file);

	/**
	 * write w/o waiting/blocking
	 */
	_EXT_ITCM int write(void *ptr, size_t size, uint64_t dropout_start);

	/**
	 * Write to the buffer but assuming there is enough space
	 */
	inline void write_no_check(void *ptr, size_t size);

	/* 512 didn't seem to work properly, 4096 should match the FAT cluster size */
	static constexpr size_t	_min_write_chunk = 4096;

	int			_fd = -1;
	uint8_t 	*_buffer = nullptr;
	const size_t	_buffer_size;
	size_t			_head = 0; ///< next position to write to
	size_t			_count = 0; ///< number of bytes in _buffer to be written
	size_t		_total_written = 0;
	bool		_should_run = false;
	bool		_running = false;
	bool 		_exit_thread = false;
	bool		_need_reliable_transfer = false;
	pthread_mutex_t		_mtx;
	pthread_cond_t		_cv;
	perf_counter_t _perf_write;
	perf_counter_t _perf_fsync;
	pthread_t _thread = 0;
};

}
}
