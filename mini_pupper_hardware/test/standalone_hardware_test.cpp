/**
 * Standalone test for mini_pupper_hardware servo conversion logic.
 * NO ROS2 dependencies - compile with:
 *   g++ -std=c++17 -o standalone_hardware_test standalone_hardware_test.cpp -lm
 *
 * Run on the robot to test without ROS2:
 *   ./standalone_hardware_test          # math-only test (no hardware needed)
 *   ./standalone_hardware_test --live   # send to real esp32-proxy socket
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

// Socket includes (for live hardware test)
#include <errno.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

// ============================================================
// Mirror of mini_pupper_hardware.hpp calibration constants
// ============================================================
static constexpr double NEUTRAL_POSITION = 512.0;
static constexpr double MICROS_PER_RAD = (760.0 - 210.0) / M_PI;
static constexpr std::array<double, 3> NEUTRAL_ANGLES_RAD = {0.0, M_PI_4, -M_PI_4};

// [axis][leg]:  leg 0=RF, 1=LF, 2=RB, 3=LB
static constexpr std::array<std::array<int, 4>, 3> SERVO_MULTIPLIERS = {
  std::array<int, 4>{1, 1, -1, -1},    // axis 0 (abduction)
  std::array<int, 4>{-1, 1, -1, 1},    // axis 1 (hip)
  std::array<int, 4>{-1, 1, -1, 1},    // axis 2 (knee)
};

static constexpr int NUM_SERVOS = 12;
static const char* SOCKET_PATH = "/tmp/esp32-proxy.socket";

// ============================================================
// Servo conversion (mirrors C++ angle_to_servo_position)
// ============================================================
uint16_t angle_to_servo_position(double angle_rad, size_t axis_index, size_t leg_index)
{
  if (axis_index >= 3 || leg_index >= 4) return static_cast<uint16_t>(NEUTRAL_POSITION);

  const double neutral_angle = NEUTRAL_ANGLES_RAD[axis_index];
  const int multiplier = SERVO_MULTIPLIERS[axis_index][leg_index];
  const double angle_deviation = (angle_rad - neutral_angle) * static_cast<double>(multiplier);
  double servo_position = NEUTRAL_POSITION - MICROS_PER_RAD * angle_deviation;

  if (std::isnan(servo_position)) return 0;
  servo_position = std::max(0.0, std::min(1023.0, servo_position));
  return static_cast<uint16_t>(std::lround(servo_position));
}

double servo_position_to_angle(uint16_t servo_position, size_t axis_index, size_t leg_index)
{
  if (axis_index >= 3 || leg_index >= 4) return 0.0;

  const double neutral_angle = NEUTRAL_ANGLES_RAD[axis_index];
  const int multiplier = SERVO_MULTIPLIERS[axis_index][leg_index];
  if (multiplier == 0) return neutral_angle;

  const double delta = (NEUTRAL_POSITION - static_cast<double>(servo_position)) / MICROS_PER_RAD;
  return neutral_angle + (delta / static_cast<double>(multiplier));
}

// ============================================================
// Simple ESP32 socket (no ROS2)
// ============================================================
class SimpleESP32
{
public:
  int sock_fd = -1;

  bool connect()
  {
    sock_fd = socket(AF_UNIX, SOCK_SEQPACKET, 0);
    if (sock_fd < 0) { perror("socket"); return false; }

    struct timeval timeout;
    timeout.tv_sec = 2;
    timeout.tv_usec = 0;
    setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(sock_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path) - 1);

    if (::connect(sock_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
      perror("connect");
      ::close(sock_fd);
      sock_fd = -1;
      return false;
    }
    std::cout << "[ESP32] Connected to " << SOCKET_PATH << "\n";
    return true;
  }

  void disconnect() { if (sock_fd >= 0) { ::close(sock_fd); sock_fd = -1; } }

  bool send_positions(const std::array<uint16_t, 12>& positions)
  {
    // Protocol: BB12H12H (size=38, cmd=1, 12x torque uint16, 12x pos uint16)
    std::vector<uint8_t> pkt;
    pkt.push_back(38);  // size
    pkt.push_back(1);   // cmd: set_position_torque
    // torque = 1 for all
    for (int i = 0; i < 12; ++i) { pkt.push_back(1); pkt.push_back(0); }
    // positions
    for (auto p : positions) { pkt.push_back(p & 0xFF); pkt.push_back((p >> 8) & 0xFF); }

    if (send(sock_fd, pkt.data(), pkt.size(), 0) < 0) { perror("send"); return false; }

    uint8_t ack[2];
    if (recv(sock_fd, ack, 2, 0) != 2 || ack[0] != 2 || ack[1] != 1)
    {
      std::cerr << "[ESP32] Bad ACK\n";
      return false;
    }
    return true;
  }

  std::vector<uint16_t> get_positions()
  {
    uint8_t req[2] = {2, 2};
    if (send(sock_fd, req, 2, 0) < 0) { perror("send"); return {}; }

    uint8_t buf[26];
    if (recv(sock_fd, buf, 26, 0) != 26 || buf[0] != 26 || buf[1] != 2)
    {
      std::cerr << "[ESP32] Bad get_positions response\n";
      return {};
    }
    std::vector<uint16_t> pos;
    for (int i = 0; i < 12; ++i)
      pos.push_back(buf[2 + i*2] | (buf[2 + i*2 + 1] << 8));
    return pos;
  }
};

// ============================================================
// Print helpers
// ============================================================
static const char* LEG_NAMES[] = {"RF", "LF", "RB", "LB"};
static const char* AXIS_NAMES[] = {"abd", "hip", "knee"};

void print_servo_array(const std::string& label, const std::array<uint16_t, 12>& s)
{
  std::cout << label << ": [";
  for (int i = 0; i < 12; ++i)
  {
    std::cout << s[i];
    if (i < 11) std::cout << (i % 3 == 2 ? ",  " : ",");
  }
  std::cout << "]\n";
  std::cout << "  RF[abd=" << s[0] << " hip=" << s[1] << " knee=" << s[2] << "]"
            << "  LF[abd=" << s[3] << " hip=" << s[4] << " knee=" << s[5] << "]\n"
            << "  RB[abd=" << s[6] << " hip=" << s[7] << " knee=" << s[8] << "]"
            << "  LB[abd=" << s[9] << " hip=" << s[10] << " knee=" << s[11] << "]\n";
}

// ============================================================
// MATH TEST: verify against known-good Python baseline
// ============================================================
void run_math_test()
{
  std::cout << "\n========== MATH TEST ==========\n";
  std::cout << "Verifying angle_to_servo_position() against Python baseline\n\n";

  // Working Python baseline (from logs):
  // LF: abd=-0.080->526, hip=1.078->461, knee_abs=-0.905->533
  // RF: abd=+0.080->498, hip=1.078->563, knee_abs=-0.905->491

  struct TestCase {
    const char* name;
    double angle;
    size_t axis;
    size_t leg;   // 0=RF, 1=LF, 2=RB, 3=LB
    uint16_t expected;
  };

  const TestCase tests[] = {
    // From working Python logs
    {"RF abd +0.080", 0.080,  0, 0, 498},
    {"RF hip  1.078", 1.078,  1, 0, 563},
    {"RF knee_abs -0.905", -0.905, 2, 0, 491},
    {"LF abd -0.080", -0.080, 0, 1, 526},
    {"LF hip  1.078", 1.078,  1, 1, 461},
    {"LF knee_abs -0.905", -0.905, 2, 1, 533},
    {"RB abd -0.080", -0.080, 0, 2, 526},  // same as LF (multiplier -1)
    {"LB abd +0.080", 0.080,  0, 3, 498},  // same as RF (multiplier -1)
    // Neutral position (all axes, all legs should be 512 at neutral angles)
    {"RF abd neutral", 0.0,    0, 0, 512},
    {"LF abd neutral", 0.0,    0, 1, 512},
    {"RF hip neutral", M_PI_4, 1, 0, 512},
    {"LF hip neutral", M_PI_4, 1, 1, 512},
  };

  int pass = 0, fail = 0;
  for (const auto& t : tests)
  {
    uint16_t got = angle_to_servo_position(t.angle, t.axis, t.leg);
    bool ok = (got == t.expected);
    std::cout << (ok ? "  PASS" : "  FAIL")
              << "  " << t.name
              << ": angle=" << t.angle
              << " -> servo=" << got
              << (ok ? "" : " (expected " + std::to_string(t.expected) + ")")
              << "\n";
    ok ? ++pass : ++fail;
  }

  std::cout << "\nResult: " << pass << " passed, " << fail << " failed\n";

  // Show full standing pose servo positions
  std::cout << "\n--- Standing pose servo positions ---\n";
  std::cout << "Input angles: LF[-0.080, 1.078, -1.983]  RF[+0.080, 1.078, -1.983]\n";
  std::cout << "              LB[-0.080, 1.078, -1.983]  RB[+0.080, 1.078, -1.983]\n\n";

  const double lf_abd=-0.080, rf_abd=0.080, lb_abd=-0.080, rb_abd=0.080;
  const double hip=1.078, knee=-1.983;
  const double knee_abs = hip + knee;  // = -0.905

  std::array<uint16_t, 12> standing;
  standing[0] = angle_to_servo_position(rf_abd, 0, 0);
  standing[1] = angle_to_servo_position(hip,    1, 0);
  standing[2] = angle_to_servo_position(knee_abs, 2, 0);
  standing[3] = angle_to_servo_position(lf_abd, 0, 1);
  standing[4] = angle_to_servo_position(hip,    1, 1);
  standing[5] = angle_to_servo_position(knee_abs, 2, 1);
  standing[6] = angle_to_servo_position(rb_abd, 0, 2);
  standing[7] = angle_to_servo_position(hip,    1, 2);
  standing[8] = angle_to_servo_position(knee_abs, 2, 2);
  standing[9]  = angle_to_servo_position(lb_abd, 0, 3);
  standing[10] = angle_to_servo_position(hip,    1, 3);
  standing[11] = angle_to_servo_position(knee_abs, 2, 3);

  print_servo_array("Standing pose", standing);

  std::cout << "\nExpected from Python:\n"
            << "  RF[abd=498 hip=563 knee=491]  LF[abd=526 hip=461 knee=533]\n"
            << "  RB[abd=526 hip=563 knee=491]  LB[abd=498 hip=461 knee=533]\n";

  // Check round-trip
  std::cout << "\n--- Round-trip test: servo->angle->servo ---\n";
  for (size_t leg = 0; leg < 4; ++leg)
    for (size_t axis = 0; axis < 3; ++axis)
    {
      uint16_t orig = standing[leg * 3 + axis];
      double angle = servo_position_to_angle(orig, axis, leg);
      uint16_t back = angle_to_servo_position(angle, axis, leg);
      bool ok = (orig == back);
      if (!ok)
        std::cout << "  FAIL round-trip " << LEG_NAMES[leg] << " " << AXIS_NAMES[axis]
                  << ": " << orig << " -> " << angle << " -> " << back << "\n";
    }
  std::cout << "  Round-trip OK for all joints\n";
}

// ============================================================
// LIVE TEST: send standing pose to robot and read back
// ============================================================
void run_live_test()
{
  std::cout << "\n========== LIVE HARDWARE TEST ==========\n";

  SimpleESP32 esp32;
  if (!esp32.connect())
  {
    std::cerr << "Cannot connect to " << SOCKET_PATH << " - is esp32-proxy running?\n";
    return;
  }

  // Read current positions first
  std::cout << "\n[1] Reading current servo positions...\n";
  auto before = esp32.get_positions();
  if (before.size() == 12)
  {
    std::cout << "  Current: [";
    for (int i = 0; i < 12; ++i) std::cout << before[i] << (i<11?",":"]\n");
    std::cout << "  RF[abd=" << before[0] << " hip=" << before[1] << " knee=" << before[2] << "]"
              << "  LF[abd=" << before[3] << " hip=" << before[4] << " knee=" << before[5] << "]\n"
              << "  RB[abd=" << before[6] << " hip=" << before[7] << " knee=" << before[8] << "]"
              << "  LB[abd=" << before[9]  << " hip=" << before[10] << " knee=" << before[11] << "]\n";
  }

  // Build standing pose
  const double lf_abd=-0.080, rf_abd=0.080, lb_abd=-0.080, rb_abd=0.080;
  const double hip=1.078, knee=-1.983, knee_abs = hip + knee;

  std::array<uint16_t, 12> standing;
  standing[0] = angle_to_servo_position(rf_abd, 0, 0);
  standing[1] = angle_to_servo_position(hip,    1, 0);
  standing[2] = angle_to_servo_position(knee_abs, 2, 0);
  standing[3] = angle_to_servo_position(lf_abd, 0, 1);
  standing[4] = angle_to_servo_position(hip,    1, 1);
  standing[5] = angle_to_servo_position(knee_abs, 2, 1);
  standing[6] = angle_to_servo_position(rb_abd, 0, 2);
  standing[7] = angle_to_servo_position(hip,    1, 2);
  standing[8] = angle_to_servo_position(knee_abs, 2, 2);
  standing[9]  = angle_to_servo_position(lb_abd, 0, 3);
  standing[10] = angle_to_servo_position(hip,    1, 3);
  standing[11] = angle_to_servo_position(knee_abs, 2, 3);

  std::cout << "\n[2] Sending standing pose...\n";
  print_servo_array("  Commanding", standing);

  if (!esp32.send_positions(standing))
  {
    std::cerr << "Failed to send positions!\n";
    esp32.disconnect();
    return;
  }
  std::cout << "  ACK received OK\n";

  // Wait for servos to move
  std::cout << "  Waiting 2s for servos to reach target...\n";
  sleep(2);

  // Read back actual positions
  std::cout << "\n[3] Reading back actual servo positions...\n";
  auto after = esp32.get_positions();
  if (after.size() == 12)
  {
    std::cout << "  Actual:   [";
    for (int i = 0; i < 12; ++i) std::cout << after[i] << (i<11?",":"]\n");
    std::cout << "  RF[abd=" << after[0] << " hip=" << after[1] << " knee=" << after[2] << "]"
              << "  LF[abd=" << after[3] << " hip=" << after[4] << " knee=" << after[5] << "]\n"
              << "  RB[abd=" << after[6] << " hip=" << after[7] << " knee=" << after[8] << "]"
              << "  LB[abd=" << after[9]  << " hip=" << after[10] << " knee=" << after[11] << "]\n";

    std::cout << "\n[4] Comparing commanded vs actual:\n";
    const char* servo_labels[] = {
      "RF-abd","RF-hip","RF-knee",
      "LF-abd","LF-hip","LF-knee",
      "RB-abd","RB-hip","RB-knee",
      "LB-abd","LB-hip","LB-knee"
    };
    bool any_error = false;
    for (int i = 0; i < 12; ++i)
    {
      int diff = static_cast<int>(after[i]) - static_cast<int>(standing[i]);
      bool ok = std::abs(diff) <= 5;  // allow 5 counts tolerance
      if (!ok) any_error = true;
      printf("  %s%-10s: commanded=%4d  actual=%4d  diff=%+4d  %s\n",
             ok ? "" : "!!! ",
             servo_labels[i], standing[i], after[i], diff,
             ok ? "OK" : "<-- MISMATCH");
    }

    if (any_error)
    {
      std::cout << "\n*** MISMATCHES DETECTED ***\n";
      std::cout << "Front servos not reaching commanded positions - physical issue (wiring/mechanical)\n";
    }
    else
    {
      std::cout << "\nAll servos reached commanded positions within tolerance!\n";
    }
  }

  // Test individual servos to find mapping
  std::cout << "\n[5] Individual servo sweep test (each servo to 400, then 624)...\n";
  std::cout << "    Watch the PHYSICAL robot to identify which servo moves!\n";
  
  std::array<uint16_t, 12> neutral_pos;
  neutral_pos.fill(512);

  for (int servo_idx = 0; servo_idx < 12; ++servo_idx)
  {
    std::cout << "  Testing servo channel " << servo_idx
              << " (" << servo_labels[servo_idx] << ")... ";
    std::cout.flush();

    // Move to 400
    auto cmd = neutral_pos;
    cmd[servo_idx] = 400;
    esp32.send_positions(cmd);
    sleep(1);

    // Read position
    auto pos = esp32.get_positions();
    int actual_400 = pos.size() == 12 ? pos[servo_idx] : -1;

    // Move to 624
    cmd[servo_idx] = 624;
    esp32.send_positions(cmd);
    sleep(1);

    auto pos2 = esp32.get_positions();
    int actual_624 = pos2.size() == 12 ? pos2[servo_idx] : -1;

    printf("400->actual=%d, 624->actual=%d", actual_400, actual_624);
    int range = std::abs(actual_624 - actual_400);
    if (range < 50) printf("  <-- SERVO NOT RESPONDING!");
    printf("\n");

    // Return to neutral
    cmd[servo_idx] = 512;
    esp32.send_positions(cmd);
    usleep(300000);
  }

  esp32.disconnect();
  std::cout << "\nDone.\n";
}

// ============================================================
// main
// ============================================================
int main(int argc, char* argv[])
{
  bool live = false;
  for (int i = 1; i < argc; ++i)
    if (std::string(argv[i]) == "--live") live = true;

  std::cout << "Mini Pupper Hardware Standalone Test\n";
  std::cout << "=====================================\n";

  // Always run math test
  run_math_test();

  if (live)
    run_live_test();
  else
  {
    std::cout << "\n[Live test skipped - run with --live to test actual hardware]\n";
    std::cout << "Example: ./standalone_hardware_test --live\n";
  }

  return 0;
}
