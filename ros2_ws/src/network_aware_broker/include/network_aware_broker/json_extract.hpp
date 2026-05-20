#pragma once

#include <regex>
#include <string>

namespace network_aware_broker
{

inline double json_number(
  const std::string & payload,
  const std::string & key,
  const double fallback)
{
  const std::regex re("\\\"" + key + "\\\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)");
  std::smatch match;
  if (std::regex_search(payload, match, re)) {
    return std::stod(match[1].str());
  }
  return fallback;
}

inline bool json_bool(
  const std::string & payload,
  const std::string & key,
  const bool fallback)
{
  const std::regex re("\\\"" + key + "\\\"\\s*:\\s*(true|false)");
  std::smatch match;
  if (std::regex_search(payload, match, re)) {
    return match[1].str() == "true";
  }
  return fallback;
}

inline std::string json_string(
  const std::string & payload,
  const std::string & key,
  const std::string & fallback)
{
  const std::regex re("\\\"" + key + "\\\"\\s*:\\s*\\\"([^\\\"]*)\\\"");
  std::smatch match;
  if (std::regex_search(payload, match, re)) {
    return match[1].str();
  }
  return fallback;
}

}  // namespace network_aware_broker
