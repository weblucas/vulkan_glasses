#ifndef VULKAN_GLASSES_STRING_UTILS_H_
#define VULKAN_GLASSES_STRING_UTILS_H_

#include <string>
#include <vector>

namespace vg_str {

// Strip leading/trailing ASCII whitespace (matches boost::trim for typical input).
inline std::string trim(const std::string& s) {
  const char* ws = " \t\n\r\f\v";
  size_t b = s.find_first_not_of(ws);
  if (b == std::string::npos) return std::string();
  size_t e = s.find_last_not_of(ws);
  return s.substr(b, e - b + 1);
}

// Split on a single delimiter without compressing consecutive delimiters
// (matches boost::split with boost::is_any_of(<one char>) and default
// token_compress_off: N delimiters yield N+1 tokens, including empty ones).
inline std::vector<std::string> split(const std::string& s, char delim) {
  std::vector<std::string> out;
  size_t start = 0;
  while (true) {
    size_t pos = s.find(delim, start);
    if (pos == std::string::npos) {
      out.push_back(s.substr(start));
      break;
    }
    out.push_back(s.substr(start, pos - start));
    start = pos + 1;
  }
  return out;
}

}  // namespace vg_str

#endif  // VULKAN_GLASSES_STRING_UTILS_H_
