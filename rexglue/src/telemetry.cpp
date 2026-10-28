// Motion telemetry: the player's place and attitude in a race, sent as UDP
// packets for a motion simulator - through SpaceMonkey (its "Burnout Revenge
// (Xerenge)" provider), which works the accelerations, rates and angles out of
// a moving transform and hands them on in Codemasters' custom UDP format to
// SimTools, SimFeedback, Sim Racing Studio, FlyPT Mover and the rest.
//
// Where it comes from. The title keeps no telemetry of its own, and the
// player car's physics object has not been found in this build yet. What is
// known is the race's camera: CGtViewport::Update makes its view window in
// sub_82353BA8 (widescreen.cpp) and sets it with RwCameraSetViewWindow
// (sub_8234ABC0) every frame. Where that camera keeps its world matrix is
// learned while driving (FindMatrix, below).
// Burnout's chase and bumper cameras are carried by the car, a little behind
// and lagging it, so the camera's path is the car's path:
//   - position: the camera's (the motion software differentiates it twice
//     for surge, sway and heave, and filters it);
//   - heading: where the camera looks, flattened onto the ground;
//   - pitch: the road's slope, from the direction of travel (the camera's own
//     downward tilt is not the car's and would hold the rig nose-down);
//   - roll: the camera's own sideways lean, scaled by telemetry_roll.
// When the car's own object is found, a hook can fill the same Sample from it
// and everything after Send stays as it is.
//
// Only a 16:9 viewport's camera counts as the race's: the menus' cameras are
// 4:3 (widescreen.cpp), the reflections' and shadows' square. Each matrix is
// checked before use - unit axes, a sane position - so a wrong guess at the
// layout sends nothing rather than nonsense; burnout.log says which (lines
// starting "telemetry:").
//
// Settings, in xerenge.conf (or XERENGE_TELEMETRY=host:port, which also turns
// it on):
//   telemetry = on             off by default
//   telemetry_host = 127.0.0.1 where SpaceMonkey runs (another PC works too)
//   telemetry_port = 20790
//   telemetry_mirror = true    RenderWare's world is right-handed; SpaceMonkey
//                              takes left-handed. Set false if the rig leans
//                              the wrong way in corners.
//   telemetry_scale = 1.0      world units to metres
//   telemetry_roll = 1.0       how much of the camera's lean is roll
//
// The packet, little-endian, 128 bytes (SpaceMonkey's BurnoutRevengeAPI.cs
// reads the same):
//    0 char[4] "XRTM"
//    4 u16  version (1)
//    6 u16  flags: 1 in a race (the rest of the packet is good), 2 inputs good
//    8 u32  sequence
//   12 u32  0
//   16 f64  time, seconds
//   24 f64  position x, y, z (metres, y up)
//   48 f64  forward x, y, z (unit)
//   72 f64  up x, y, z (unit)
//   96 f32  speed, m/s (this side's estimate, for display)
//  100 f32  throttle, brake, steering, rpm, gear - -1 when not known
//  120 f32  0, 0
#include "telemetry.h"

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <vector>

#include <rex/hook.h>
#include <rex/logging.h>
#include <rex/ppc.h>
#include <rex/system/kernel_state.h>
#include <rex/system/xmemory.h>
#include "guest_memory.h"
#include "xerenge_settings.h"

namespace {

using Clock = std::chrono::steady_clock;

struct Vec3 {
  double x = 0, y = 0, z = 0;
};

Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 operator*(Vec3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
double Dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 Cross(Vec3 a, Vec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
double Length(Vec3 a) { return std::sqrt(Dot(a, a)); }
Vec3 Normalized(Vec3 a) {
  const double len = Length(a);
  return len > 1e-9 ? a * (1.0 / len) : Vec3{};
}
double Axis(Vec3 v, int i) { return i == 0 ? v.x : i == 1 ? v.y : v.z; }

// --- settings ----------------------------------------------------------------

struct Config {
  bool enabled = false;
  std::string host = "127.0.0.1";
  int port = 20790;
  bool mirror = true;
  double scale = 1.0;
  double roll = 1.0;
};

double ParseNumber(const std::string& text, double fallback) {
  char* end = nullptr;
  const double value = std::strtod(text.c_str(), &end);
  return end != text.c_str() && std::isfinite(value) ? value : fallback;
}

const Config& Settings() {
  static const Config config = [] {
    Config c;
    namespace s = xerenge::settings;
    const std::string on = s::ConfString("telemetry", "off");
    c.enabled = on == "on" || on == "true" || on == "1";
    c.host = s::ConfString("telemetry_host", c.host);
    c.port = int(ParseNumber(s::ConfString("telemetry_port", "20790"), 20790));
    c.mirror = s::ConfString("telemetry_mirror", "true") != "false";
    c.scale = ParseNumber(s::ConfString("telemetry_scale", "1.0"), 1.0);
    c.roll = std::clamp(ParseNumber(s::ConfString("telemetry_roll", "1.0"), 1.0), 0.0, 2.0);
    // XERENGE_TELEMETRY=host:port (or just a port) turns it on from outside.
    if (const char* env = std::getenv("XERENGE_TELEMETRY"); env && *env) {
      std::string value = env;
      c.enabled = value != "off" && value != "0";
      const auto colon = value.rfind(':');
      if (colon != std::string::npos) {
        c.host = value.substr(0, colon);
        c.port = int(ParseNumber(value.substr(colon + 1), c.port));
      } else if (std::isdigit(static_cast<unsigned char>(value[0]))) {
        c.port = int(ParseNumber(value, c.port));
      }
    }
    if (c.port <= 0 || c.port > 65535) {
      c.port = 20790;
    }
    if (c.enabled) {
      REXLOG_INFO("telemetry: on, to {}:{} (mirror {}, scale {:.3f}, roll {:.2f})", c.host, c.port,
                  c.mirror, c.scale, c.roll);
    }
    return c;
  }();
  return config;
}

// --- the socket --------------------------------------------------------------

class Sender {
 public:
  bool Open(const Config& c) {
    if (opened_) {
      return ok_;
    }
    opened_ = true;
#if defined(_WIN32)
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
#endif
    socket_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (!Valid()) {
      REXLOG_WARN("telemetry: no UDP socket; nothing will be sent");
      return false;
    }
#if defined(_WIN32)
    u_long nonblocking = 1;
    ioctlsocket(socket_, FIONBIO, &nonblocking);
#endif
    std::memset(&to_, 0, sizeof(to_));
    to_.sin_family = AF_INET;
    to_.sin_port = htons(uint16_t(c.port));
    if (inet_pton(AF_INET, c.host.c_str(), &to_.sin_addr) != 1) {
      REXLOG_WARN("telemetry: '{}' is not an IPv4 address; nothing will be sent", c.host);
      return false;
    }
    ok_ = true;
    return true;
  }

  void Send(const uint8_t* data, size_t size) {
    if (!ok_) {
      return;
    }
#if defined(_WIN32)
    ::sendto(socket_, reinterpret_cast<const char*>(data), int(size), 0,
             reinterpret_cast<const sockaddr*>(&to_), sizeof(to_));
#else
    ::sendto(socket_, data, size, MSG_DONTWAIT, reinterpret_cast<const sockaddr*>(&to_), sizeof(to_));
#endif
  }

 private:
#if defined(_WIN32)
  SOCKET socket_ = INVALID_SOCKET;
  bool Valid() const { return socket_ != INVALID_SOCKET; }
#else
  int socket_ = -1;
  bool Valid() const { return socket_ >= 0; }
#endif
  sockaddr_in to_{};
  bool opened_ = false;
  bool ok_ = false;
};

// --- the packet --------------------------------------------------------------

constexpr size_t kPacketSize = 128;
constexpr uint16_t kVersion = 1;
constexpr uint16_t kFlagInRace = 1;

struct Sample {
  bool in_race = false;
  double time = 0;
  Vec3 position, forward, up;
  float speed = 0;
};

template <typename T>
void Put(uint8_t* packet, size_t offset, T value) {
  std::memcpy(packet + offset, &value, sizeof(T));  // every host Xerenge runs on is little-endian
}

void PutVec(uint8_t* packet, size_t offset, Vec3 v) {
  Put(packet, offset, v.x);
  Put(packet, offset + 8, v.y);
  Put(packet, offset + 16, v.z);
}

// --- the state ---------------------------------------------------------------

std::mutex g_mutex;
Sender g_sender;
uint32_t g_sequence = 0;

thread_local uint32_t g_viewport = 0;
thread_local bool g_race_viewport = false;

// Which way is up, found from the race camera's own up over its first frames
// (RenderWare games differ: y for most, z for some).
std::array<double, 3> g_up_sum{};
int g_up_samples = 0;
int g_up_axis = -1;
double g_up_sign = 1;

// The last race sample, for the slope and for leaving out repeated frames.
bool g_have_last = false;
Vec3 g_last_world;
Vec3 g_last_at;
double g_last_time = 0;
double g_last_sent = 0;
Vec3 g_velocity;
double g_pitch = 0;
Clock::time_point g_last_race{};
Clock::time_point g_last_idle{};
bool g_told_camera = false;

double Seconds() {
  static const auto start = Clock::now();
  return std::chrono::duration<double>(Clock::now() - start).count();
}

float GuestFloat(const uint8_t* base, uint32_t address) {
  const uint32_t bits = xerenge::LoadGuestU32(base, address);
  float value;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

Vec3 GuestVec(const uint8_t* base, uint32_t address) {
  return {GuestFloat(base, address), GuestFloat(base, address + 4), GuestFloat(base, address + 8)};
}

bool Unit(Vec3 v) {
  const double len = Length(v);
  return std::isfinite(len) && len > 0.9 && len < 1.1;
}

void Send(const Sample& s) {
  const Config& c = Settings();
  if (!g_sender.Open(c)) {
    return;
  }
  uint8_t packet[kPacketSize] = {};
  std::memcpy(packet, "XRTM", 4);
  Put(packet, 4, kVersion);
  Put(packet, 6, uint16_t(s.in_race ? kFlagInRace : 0));
  Put(packet, 8, g_sequence++);
  Put(packet, 16, s.time);
  PutVec(packet, 24, s.position);
  PutVec(packet, 48, s.forward);
  PutVec(packet, 72, s.up);
  Put(packet, 96, s.speed);
  for (size_t offset = 100; offset <= 116; offset += 4) {
    Put(packet, offset, -1.0f);  // throttle, brake, steering, rpm, gear: not known yet
  }
  g_sender.Send(packet, sizeof(packet));
}

// The world's up axis from the camera's, then the world turned so that it is
// y - a proper rotation, then mirrored when asked (left-handed for SpaceMonkey).
Vec3 ToOutput(Vec3 v) {
  const Config& c = Settings();
  Vec3 out;
  switch (g_up_axis) {
    case 0:  // x up: the axes turned round once, x to y, y to z, z to x
      out = {Axis(v, 2), Axis(v, 0), Axis(v, 1)};
      break;
    case 2:  // z up: rotation about x by -90 degrees
      out = {v.x, v.z, -v.y};
      break;
    default:
      out = v;
      break;
  }
  if (g_up_sign < 0) {
    // Up pointing down the axis: half a turn about z.
    out = {-out.x, -out.y, out.z};
  }
  if (c.mirror) {
    out.x = -out.x;
  }
  return out;
}

void LearnUp(Vec3 camera_up) {
  if (g_up_axis >= 0) {
    return;
  }
  g_up_sum[0] += camera_up.x;
  g_up_sum[1] += camera_up.y;
  g_up_sum[2] += camera_up.z;
  if (++g_up_samples < 30) {
    return;
  }
  int axis = 0;
  for (int i = 1; i < 3; ++i) {
    if (std::fabs(g_up_sum[i]) > std::fabs(g_up_sum[axis])) {
      axis = i;
    }
  }
  g_up_axis = axis;
  g_up_sign = g_up_sum[axis] < 0 ? -1.0 : 1.0;
  REXLOG_INFO("telemetry: the world's up is {}{}", g_up_sign < 0 ? "-" : "+", "xyz"[axis]);
}

// Whether the title can read [address, address + bytes). The camera is looked
// for by following what might be pointers, and one that lands on an unmapped
// page would take the process down.
bool Readable(uint32_t address, uint32_t bytes) {
  if (address < 0x10000000u || bytes == 0) {
    return false;
  }
  const uint32_t last = address + bytes - 1;
  if (last < address) {
    return false;
  }
  auto* kernel_state = REX_KERNEL_STATE();
  auto* memory = kernel_state ? kernel_state->memory() : nullptr;
  if (!memory) {
    return false;
  }
  auto* heap = memory->LookupHeap(address);
  if (!heap || memory->LookupHeap(last) != heap) {
    return false;
  }
  return heap->QueryRangeAccess(address, last) != rex::memory::PageAccess::kNoAccess;
}

double AbsDot(Vec3 a, Vec3 b) { return std::fabs(Dot(a, b)); }

// Where the race camera's world matrix is. sub_8234ABC0 is RwCameraSetViewWindow
// all right - it stores the view window at +88 and its reciprocal at +96 of
// r3, which CGtViewport::Update (sub_82353BA8) takes from the viewport's +88 -
// but this build's camera is not laid out as RenderWare 3's: its first word is
// a pointer and no RwFrame follows it (the log's "type 130" was that pointer's
// top byte). So the matrix is learned instead of assumed.
//
// A candidate is any place, in the camera, in the viewport or in an object
// either of them points to, that holds a world matrix as RenderWare keeps one:
// right +0, up +16, at +32, pos +48, three unit axes at right angles and a
// sane position. A chase camera looks where it goes, so the one whose position
// moves along its own "at", frame after frame, is the camera's world matrix -
// a view matrix (the inverse), an identity or a parked object never does. That
// one is used; if it stops looking like a matrix, the search starts over.

// A world matrix at m (the caller has checked that 64 bytes can be read).
bool WorldMatrix(const uint8_t* base, uint32_t m) {
  // Most places fail on the first axis; the rest is read only when it holds.
  const Vec3 right = GuestVec(base, m);
  if (!Unit(right)) {
    return false;
  }
  const Vec3 up = GuestVec(base, m + 16);
  if (!Unit(up) || AbsDot(right, up) >= 0.05) {
    return false;
  }
  const Vec3 at = GuestVec(base, m + 32);
  if (!Unit(at) || AbsDot(right, at) >= 0.05 || AbsDot(up, at) >= 0.05) {
    return false;
  }
  const Vec3 pos = GuestVec(base, m + 48);
  return std::isfinite(Length(pos)) && Length(pos) < 1.0e6;
}

struct Candidate {
  uint32_t address = 0;
  bool have = false;
  Vec3 last_pos;
  double last_time = 0;
  int along = 0;    // frames it moved the way it looks
  int against = 0;  // frames it moved some other way
};

std::vector<Candidate> g_candidates;
uint32_t g_matrix = 0;
double g_last_gather = -100.0;
double g_last_report = -100.0;
constexpr size_t kMaxCandidates = 256;
constexpr uint32_t kObjectBytes = 0x400;

// How many bytes from object on can be read, up to kObjectBytes.
uint32_t ReadableSpan(uint32_t object) {
  uint32_t span = kObjectBytes;
  while (span >= 64 && !Readable(object, span)) {
    span /= 2;
  }
  return span >= 64 ? span : 0;
}

void AddCandidate(uint32_t address) {
  for (const Candidate& c : g_candidates) {
    if (c.address == address) {
      return;
    }
  }
  if (g_candidates.size() < kMaxCandidates) {
    Candidate c;
    c.address = address;
    g_candidates.push_back(c);
  }
}

void ScanObject(const uint8_t* base, uint32_t object, bool follow) {
  const uint32_t span = ReadableSpan(object);
  if (span == 0) {
    return;
  }
  for (uint32_t offset = 0; offset + 64 <= span; offset += 4) {
    if (WorldMatrix(base, object + offset)) {
      AddCandidate(object + offset);
    }
  }
  if (!follow) {
    return;
  }
  for (uint32_t offset = 0; offset < span; offset += 4) {
    const uint32_t p = xerenge::LoadGuestU32(base, object + offset);
    if (p >= 0x10000000u && p != object && (p & 3) == 0) {
      ScanObject(base, p, false);
    }
  }
}

// One frame's look at every candidate; the matrix once one has shown itself.
uint32_t LearnMatrix(const uint8_t* base, uint32_t camera, uint32_t viewport) {
  const double now = Seconds();
  // Once a second at first, every five once there is something to watch.
  if (now - g_last_gather >= (g_candidates.empty() ? 1.0 : 5.0)) {
    g_last_gather = now;
    ScanObject(base, camera, true);
    if (viewport != 0) {
      ScanObject(base, viewport, true);
    }
  }
  for (Candidate& c : g_candidates) {
    if (!Readable(c.address, 64) || !WorldMatrix(base, c.address)) {
      c.have = false;
      continue;
    }
    const Vec3 at = GuestVec(base, c.address + 32);
    const Vec3 pos = GuestVec(base, c.address + 48);
    if (c.have) {
      const double dt = now - c.last_time;
      const Vec3 d = pos - c.last_pos;
      const double dist = Length(d);
      if (dt > 0.0 && dt < 0.25 && dist / dt > 2.0 && dist / dt < 150.0) {
        if (Dot(d * (1.0 / dist), at) > 0.7) {
          ++c.along;
        } else {
          ++c.against;
        }
      }
    }
    c.have = true;
    c.last_pos = pos;
    c.last_time = now;
    if (c.along >= 40 && c.along > 4 * c.against) {
      REXLOG_INFO("telemetry: camera matrix found at {:08X} (camera {:08X}, viewport {:08X}; "
                  "{} frames along its view, {} not)",
                  c.address, camera, viewport, c.along, c.against);
      return c.address;
    }
  }
  if (now - g_last_report >= 10.0) {
    g_last_report = now;
    std::string best;
    std::vector<const Candidate*> order;
    for (const Candidate& c : g_candidates) {
      order.push_back(&c);
    }
    std::sort(order.begin(), order.end(),
              [](const Candidate* a, const Candidate* b) { return a->along > b->along; });
    for (size_t i = 0; i < order.size() && i < 5; ++i) {
      char text[48];
      std::snprintf(text, sizeof(text), " %08X(%d/%d)", order[i]->address, order[i]->along,
                    order[i]->against);
      best += text;
    }
    REXLOG_WARN("telemetry: no camera matrix yet - {} candidates from camera {:08X} and "
                "viewport {:08X}; best (along/not):{}; nothing is sent until one moves "
                "the way it looks, so drive",
                g_candidates.size(), camera, viewport, best.empty() ? " none" : best);
  }
  return 0;
}

uint32_t FindMatrix(const uint8_t* base, uint32_t camera, uint32_t viewport) {
  if (g_matrix != 0) {
    if (Readable(g_matrix, 64) && WorldMatrix(base, g_matrix)) {
      return g_matrix;
    }
    REXLOG_WARN("telemetry: the camera matrix at {:08X} is gone; looking again", g_matrix);
    g_matrix = 0;
    g_candidates.clear();
  }
  g_matrix = LearnMatrix(base, camera, viewport);
  if (g_matrix != 0) {
    g_candidates.clear();
  }
  return g_matrix;
}

// The race camera's matrix this frame, made into a car-like transform.
void RaceCamera(const uint8_t* base, uint32_t camera) {
  const Config& c = Settings();
  const uint32_t matrix = FindMatrix(base, camera, g_viewport);
  if (matrix == 0) {
    return;
  }
  // The world matrix: up +16, at +32, pos +48 (right, +0, is not needed).
  const Vec3 cam_up = GuestVec(base, matrix + 16);
  const Vec3 at = GuestVec(base, matrix + 32);
  const Vec3 pos = GuestVec(base, matrix + 48);
  if (!g_told_camera) {
    g_told_camera = true;
    REXLOG_INFO("telemetry: race camera {:08X}, matrix {:08X}, viewport {:08X}", camera, matrix,
                g_viewport);
  }
  LearnUp(cam_up);
  if (g_up_axis < 0) {
    return;
  }

  const double now = Seconds();
  const Vec3 world = ToOutput(pos * c.scale);
  const Vec3 look = ToOutput(at);
  const Vec3 lean = ToOutput(cam_up);

  // The same picture again (a frame drawn twice, the game paused a moment):
  // sending it would read as a sudden stop. A still car is still sent, a
  // twentieth of a second on.
  if (g_have_last && Length(world - g_last_world) == 0.0 && Length(look - g_last_at) == 0.0 &&
      now - g_last_sent < 0.05) {
    return;
  }

  const double dt = g_have_last ? now - g_last_time : 0.0;
  if (g_have_last && dt > 0.0 && dt < 0.25) {
    const Vec3 v = (world - g_last_world) * (1.0 / dt);
    // A cut (crash camera, restart): no velocity from it.
    if (Length(v) < 150.0) {
      const double k = std::min(1.0, dt / 0.15);
      g_velocity = g_velocity + (v - g_velocity) * k;
    } else {
      g_velocity = {};
    }
  } else {
    g_velocity = {};
  }
  g_have_last = true;
  g_last_world = world;
  g_last_at = look;
  g_last_time = now;

  const Vec3 up_axis{0, 1, 0};
  Vec3 heading{look.x, 0, look.z};
  if (Length(heading) < 0.3) {
    return;  // looking straight down: no heading to speak of
  }
  heading = Normalized(heading);
  const Vec3 side = Normalized(Cross(up_axis, heading));

  // Pitch: the road's slope along the way the car goes.
  const Vec3 flat{g_velocity.x, 0, g_velocity.z};
  const double flat_speed = Length(flat);
  double slope = 0.0;
  if (flat_speed > 4.0 && Dot(flat, heading) > 0.0) {
    slope = std::atan(std::clamp(g_velocity.y / flat_speed, -0.6, 0.6));
  }
  const double k = std::min(1.0, std::max(dt, 0.0) / 0.2);
  g_pitch += (slope - g_pitch) * k;

  // Roll: the camera's lean to the side, against the up axis.
  const double roll = std::atan2(Dot(lean, side), Dot(lean, up_axis)) * c.roll;

  const Vec3 forward = heading * std::cos(g_pitch) + up_axis * std::sin(g_pitch);
  const Vec3 pitched_up = up_axis * std::cos(g_pitch) - heading * std::sin(g_pitch);
  const Vec3 up = Normalized(pitched_up * std::cos(roll) + side * std::sin(roll));

  Sample s;
  s.in_race = true;
  s.time = now;
  s.position = world;
  s.forward = Normalized(forward);
  s.up = up;
  s.speed = float(Length(g_velocity));
  Send(s);
  g_last_sent = now;
  g_last_race = Clock::now();
}

}  // namespace

namespace xerenge::telemetry {

void EnterViewport(uint32_t viewport, float aspect) {
  if (!Settings().enabled) {
    return;
  }
  g_viewport = viewport;
  g_race_viewport = aspect > 1.7f && aspect < 1.85f;
  if (g_race_viewport) {
    return;
  }
  // Out of a race (the menus' 4:3 cameras): "not in a race", ten times a
  // second, so the motion software eases the rig back to rest.
  std::lock_guard lock(g_mutex);
  const auto now = Clock::now();
  if (now - g_last_race > std::chrono::milliseconds(250) &&
      now - g_last_idle > std::chrono::milliseconds(100)) {
    g_last_idle = now;
    g_have_last = false;
    Sample idle;
    idle.time = Seconds();
    idle.forward = {0, 0, 1};
    idle.up = {0, 1, 0};
    Send(idle);
  }
}

void LeaveViewport() {
  g_viewport = 0;
  g_race_viewport = false;
}

}  // namespace xerenge::telemetry

// RwCameraSetViewWindow(camera, view window): inside the race viewport's
// update, r3 is the race camera.
extern "C" void __imp__sub_8234ABC0(PPCContext& __restrict, uint8_t*);
REX_HOOK_RAW(sub_8234ABC0) {
  const uint32_t camera = ctx.r3.u32;
  if (g_race_viewport && camera != 0) {
    std::lock_guard lock(g_mutex);
    RaceCamera(base, camera);
  }
  __imp__sub_8234ABC0(ctx, base);
}
