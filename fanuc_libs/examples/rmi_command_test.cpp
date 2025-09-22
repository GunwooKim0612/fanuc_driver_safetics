// rmi_example_progress_numbered.cpp
#include <chrono>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "rmi/packets.hpp"
#include "rmi/rmi.hpp"

using namespace std::chrono_literals;

struct Logger
{
  std::ofstream ofs;
  explicit Logger(const std::string& path) : ofs(path, std::ios::out | std::ios::trunc)
  {
    if (!ofs)
      throw std::runtime_error("로그 파일을 열 수 없습니다: " + path);
  }
  static std::string now()
  {
    using clock = std::chrono::system_clock;
    auto t = clock::to_time_t(clock::now());
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    std::ostringstream ss;
    ss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    return ss.str();
  }
  void line(const std::string& s)
  {
    ofs << "[" << now() << "] " << s << "\n";
    ofs.flush();
  }
};

struct StepResult
{
  int index;
  std::string name;
  bool ok;
  std::string detail;
};

int main(int argc, char** argv)
{
  const std::string robot_ip = (argc > 1) ? argv[1] : "192.168.1.10";
  const uint16_t rmi_port = (argc > 2) ? static_cast<uint16_t>(std::stoi(argv[2])) : 16001;
  const std::string out_path = (argc > 3) ? argv[3] : "results_rmi_test.txt";
  const double timeout_s = 3.0;

  Logger log(out_path);
  auto print = [&](const std::string& s) {
    std::cout << s << std::flush;
    log.line(s);
  };

  std::vector<StepResult> results;
  int step_idx = 0;
  auto run_step = [&](const std::string& name, std::function<void()> fn) {
    ++step_idx;
    std::ostringstream hdr;
    hdr << "[" << step_idx << "] " << name;
    std::string prefix = hdr.str();
    print(prefix + " ... 진행중");
    try
    {
      fn();
      print("\r" + prefix + " ... 성공\n");
      results.push_back({ step_idx, name, true, "" });
    }
    catch (const std::exception& e)
    {
      print("\r" + prefix + " ... 실패: " + e.what() + "\n");
      results.push_back({ step_idx, name, false, e.what() });
    }
  };

  try
  {
    rmi::RMIConnection conn(robot_ip, rmi_port);

    // [2.2.1] FRC_Connect
    run_step("[2.2.1] FRC_Connect (세션 시작)", [&]() { (void)conn.connect(timeout_s); });

    // [2.3.7] FRC_GetStatus
    run_step("[2.3.7] FRC_GetStatus (컨트롤러 상태)", [&]() { (void)conn.getStatus(timeout_s); });

    // [2.3.20] FRC_Reset
    run_step("[2.3.20] FRC_Reset (에러 리셋)", [&]() { (void)conn.reset(timeout_s); });

    // [2.3.1] FRC_Initialize
    run_step("[2.3.1] FRC_Initialize (RMI 초기화)", [&]() { (void)conn.initializeRemoteMotion(timeout_s); });

    // [2.3.16] FRC_SetOverRide
    run_step("[2.3.16] FRC_SetOverRide=50%", [&]() { (void)conn.setSpeedOverride(50, timeout_s); });

    // [2.3.15] FRC_ReadJointAngles
    run_step("[2.3.15] FRC_ReadJointAngles (현재 조인트)",
             [&]() { (void)conn.readJointAngles(std::nullopt, timeout_s); });

    // [2.3.21] FRC_ReadTCPSpeed  (라이브러리: GetTCPSpeedPacket 사용)
    run_step("[2.3.21] FRC_ReadTCPSpeed (현재 TCP 속도)", [&]() {
      rmi::GetTCPSpeedPacket::Request req{};
      (void)conn.sendRMIPacket<rmi::GetTCPSpeedPacket>(req, timeout_s);
    });

    // [2.3.12] FRC_ReadDIN
    run_step("[2.3.12] FRC_ReadDIN (DI[1])", [&]() { (void)conn.readDigitalInputPort(1, timeout_s); });

    // [2.3.13] FRC_WriteDOUT
    run_step("[2.3.13] FRC_WriteDOUT (DO[1] ON→OFF)", [&]() {
      (void)conn.writeDigitalOutputPort(1, true, timeout_s);
      std::this_thread::sleep_for(100ms);
      (void)conn.writeDigitalOutputPort(1, false, timeout_s);
    });

    run_step("2.3.14 FRC_ReadCartesianPosition (현재 TCP 자세/자세정보)", [&]() {
      (void)conn.getCartesianPosition(std::nullopt, timeout_s);
    });

    // [2.3.19] FRC_WritePositionRegister & [2.3.18] FRC_ReadPositionRegister
    run_step("[2.3.19/18] PR[1] 쓰기/읽기 (UFRAME/UTOOL 포함)", [&]() {
      rmi::ConfigurationData cfg{};
      cfg.UFrameNumber = 1;
      cfg.UToolNumber = 1;
      rmi::PositionData pos{};
      pos.X = 0;
      pos.Y = 0;
      pos.Z = 0;
      pos.W = 0;
      pos.P = 0;
      pos.R = 0;
      (void)conn.writePositionRegister(1, cfg, pos, timeout_s);
      (void)conn.readPositionRegister(1, timeout_s);
    });

    // [2.4.4] FRC_WaitTime
    run_step("[2.4.4] FRC_WaitTime (0.2초)", [&]() {
      rmi::WaitForTimePacket::Request req{};
      req.Time = 0.2f;
      (void)conn.sendRMIPacket<rmi::WaitForTimePacket>(req, timeout_s);
    });

    // [2.4.2] FRC_SetUFrame  /  [2.4.3] FRC_SetUTool
    run_step("[2.4.2/2.4.3] FRC_SetUFrame=1 / FRC_SetUTool=1", [&]() {
      rmi::SetUFramePacket::Request uf{};
      uf.FrameNumber = 1;
      (void)conn.sendRMIPacket<rmi::SetUFramePacket>(uf, timeout_s);
      rmi::SetToolFramePacket::Request ut{};
      ut.ToolNumber = 1;
      (void)conn.sendRMIPacket<rmi::SetToolFramePacket>(ut, timeout_s);
    });

    // [2.4.13] FRC_JointMotionJRep  (간단 예: FINE)
    run_step("[2.4.13] FRC_JointMotionJRep (JREP, FINE)", [&]() {
      rmi::JointMotionJRepPacket::Request jr{};
      jr.JointAngle = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
      jr.SpeedType = "Percent";
      jr.Speed = 5;
      jr.TermType = "FINE";
      (void)conn.sendRMIPacket<rmi::JointMotionJRepPacket>(jr, timeout_s);
    });

    // [2.3.2] FRC_Abort
    run_step("[2.3.2] FRC_Abort (RMI_MOVE 중지)", [&]() { (void)conn.abort(timeout_s); });

    // [2.2.2] FRC_Disconnect
    run_step("[2.2.2] FRC_Disconnect (세션 종료)", [&]() { (void)conn.disconnect(timeout_s); });

    // 요약 출력/저장
    std::cout << "\n=== 요약 ===\n";
    for (const auto& r : results)
      std::cout << "[" << r.index << "] " << r.name << " : " << (r.ok ? "성공" : "실패") << "\n";
    std::cout << "\n결과 파일: " << out_path << std::endl;

    log.line("=== SUMMARY ===");
    for (const auto& r : results)
      log.line("[" + std::to_string(r.index) + "] " + r.name + " : " + (r.ok ? "PASS" : "FAIL"));
  }
  catch (const std::exception& e)
  {
    std::cerr << "\n치명적 오류: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}
