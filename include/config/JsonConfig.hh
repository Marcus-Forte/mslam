#pragma once

#include "IConfig.hh"
#include <filesystem>

namespace mslam {

class JsonConfig : public IConfig {
public:
  JsonConfig(const std::filesystem::path &config_file);
  void load() override;

private:
  std::filesystem::path config_file_;
};

} // namespace mslam