#pragma once

#include <fstream>
#include <string>

namespace network_aware_broker
{

class DeterministicLog
{
public:
  DeterministicLog() = default;

  void open(const std::string & path)
  {
    if (path.empty()) {
      return;
    }
    stream_.open(path, std::ios::out | std::ios::trunc);
    if (stream_.is_open()) {
      stream_ << "wall_time_sec,event,seq,linear_x,angular_z,latency_ms,rtt_ms,jitter_ms,packet_error_rate,snr_db,rtf,los,path_loss_db,source\n";
    }
  }

  bool enabled() const
  {
    return stream_.is_open();
  }

  void write_line(const std::string & line)
  {
    if (stream_.is_open()) {
      stream_ << line << "\n";
      stream_.flush();
    }
  }

private:
  std::ofstream stream_;
};

}  // namespace network_aware_broker
