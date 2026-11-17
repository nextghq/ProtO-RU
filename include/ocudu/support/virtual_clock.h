// SPDX-FileCopyrightText: Copyright (C) 2026 NextG Systems
// SPDX-License-Identifier: BSD-3-Clause-Open-MPI

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

namespace ocudu {

/// \brief Shared-memory layout of the virtual clock.
///
/// A channel emulator that runs slower than real time, such as a ray-traced digital twin, owns the sample clock of
/// every radio it connects. It publishes the number of samples it has served, and the processes whose timing would
/// otherwise follow the host clock (Open Fronthaul timing in the O-DU and the O-RU) read their time from it instead.
/// Little endian, written by one process and read by any number.
struct virtual_clock_layout {
  /// Magic "OCUVCLK1".
  char magic[8];
  /// Sample rate in hertz.
  double srate_hz;
  /// Host real time (CLOCK_REALTIME) in nanoseconds that corresponds to sample zero.
  int64_t epoch_ns;
  /// Number of samples served so far.
  std::atomic<uint64_t> nof_samples;
};

/// \brief Virtual clock read from shared memory.
///
/// Enabled by the environment variable \c OCUDU_VIRTUAL_CLOCK, which names the shared-memory file. Without it, the
/// functions below return the host clock.
class virtual_clock
{
public:
  /// Returns the virtual clock named by \c OCUDU_VIRTUAL_CLOCK, or nullptr when the variable is not set.
  static const virtual_clock* get()
  {
    static const virtual_clock* instance = open_from_environment();
    return instance;
  }

  /// Returns the virtual time as a CLOCK_REALTIME equivalent.
  ::timespec now() const
  {
    const uint64_t samples = layout->nof_samples.load(std::memory_order_acquire);
    // Integer seconds plus fractional samples keep nanosecond precision for long runs.
    const uint64_t srate   = static_cast<uint64_t>(layout->srate_hz);
    const int64_t  ns      = layout->epoch_ns + static_cast<int64_t>((samples / srate) * 1000000000ULL) +
                       static_cast<int64_t>((samples % srate) * 1000000000ULL / srate);
    ::timespec ts;
    ts.tv_sec  = ns / 1000000000LL;
    ts.tv_nsec = ns % 1000000000LL;
    return ts;
  }

private:
  explicit virtual_clock(const virtual_clock_layout* layout_) : layout(layout_) {}

  static const virtual_clock* open_from_environment()
  {
    const char* path = std::getenv("OCUDU_VIRTUAL_CLOCK");
    if (path == nullptr || path[0] == '\0') {
      return nullptr;
    }
    const int fd = ::open(path, O_RDONLY);
    if (fd < 0) {
      std::fprintf(stderr, "OCUDU_VIRTUAL_CLOCK: cannot open '%s'\n", path);
      std::abort();
    }
    void* addr = ::mmap(nullptr, sizeof(virtual_clock_layout), PROT_READ, MAP_SHARED, fd, 0);
    ::close(fd);
    if (addr == MAP_FAILED) {
      std::fprintf(stderr, "OCUDU_VIRTUAL_CLOCK: cannot map '%s'\n", path);
      std::abort();
    }
    const auto* layout = static_cast<const virtual_clock_layout*>(addr);
    if (std::memcmp(layout->magic, "OCUVCLK1", sizeof(layout->magic)) != 0 || !(layout->srate_hz > 0.0)) {
      std::fprintf(stderr, "OCUDU_VIRTUAL_CLOCK: '%s' is not a virtual clock\n", path);
      std::abort();
    }
    std::fprintf(stderr, "Timing follows the virtual clock in '%s' (%.2f Msps)\n", path, layout->srate_hz / 1e6);
    return new virtual_clock(layout);
  }

  const virtual_clock_layout* layout;
};

/// Returns the host real time (CLOCK_REALTIME), or the virtual time when a virtual clock is enabled.
inline ::timespec realtime_clock_now()
{
  if (const virtual_clock* clock = virtual_clock::get()) {
    return clock->now();
  }
  ::timespec ts;
  ::clock_gettime(CLOCK_REALTIME, &ts);
  return ts;
}

/// Returns the system clock time, or the virtual time when a virtual clock is enabled.
inline std::chrono::system_clock::time_point system_clock_now()
{
  if (const virtual_clock* clock = virtual_clock::get()) {
    const ::timespec ts = clock->now();
    return std::chrono::system_clock::time_point(std::chrono::duration_cast<std::chrono::system_clock::duration>(
        std::chrono::seconds(ts.tv_sec) + std::chrono::nanoseconds(ts.tv_nsec)));
  }
  return std::chrono::system_clock::now();
}

} // namespace ocudu
