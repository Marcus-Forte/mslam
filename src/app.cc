#include "ConsoleLogger.hh"
#include "RemoteScan.hh"
#include <chrono>
#include <thread>

int main(int argc, char **argv) {
  RemoteScan rscan("192.168.1.10:50051");
  rscan.Start();
  ConsoleLogger logger;

  while (true) {
    const auto scan = rscan.getScan();
    if (scan.points.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    logger.log(ILog::Level::INFO, "{} nr scans: {}", scan.timestamp,
               scan.points.size());
  }
}