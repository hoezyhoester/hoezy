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
// (sub_8234ABC0) every frame; r3 there leads to the RwCamera (FindFrame). A
// RenderWare camera is an RwObjectHasFrame, so its frame is at +4, and the
// frame's world matrix (its LTM) at +80: right, up, at and pos, each a vector of three floats and a pad.
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

// An RwFrame (RwObject type 0) whose world matrix (LTM, +80: right, up, at,
// pos) is a rotation and a sane position.
bool UsableFrame(const uint8_t* base, uint32_t frame) {
  if (!Readable(frame, 144) || *xerenge::GuestPointer(base, frame) != 0) {
    return false;
  }
  const Vec3 right = GuestVec(base, frame + 80);
  const Vec3 up = GuestVec(base, frame + 96);
  const Vec3 at = GuestVec(base, frame + 112);
  const Vec3 pos = GuestVec(base, frame + 128);
  return Unit(right) && Unit(up) && Unit(at) && AbsDot(right, up) < 0.1 &&
         AbsDot(right, at) < 0.1 && AbsDot(up, at) < 0.1 && std::isfinite(Length(pos)) &&
         Length(pos) < 1.0e6;
}

// An RwCamera (RwObject type 4, its frame at +4) with a usable frame: the
// frame, or 0.
uint32_t CameraFrame(const uint8_t* base, uint32_t camera) {
  if (!Readable(camera, 8) || *xerenge::GuestPointer(base, camera) != 4) {
    return 0;
  }
  const uint32_t frame = xerenge::LoadGuestU32(base, camera + 4);
  return UsableFrame(base, frame) ? frame : 0;
}

// What sub_8234ABC0 gets in r3 is not always the RwCamera itself: on the
// retail 360 build it is an object of the title's own whose first word is a
// vtable (the log's "type 130" is that pointer's top byte, 0x82). So the
// camera is looked for: r3 itself, else an RwCamera that a pointer in r3's
// first 512 bytes leads to, else an RwFrame one leads to directly. The way
// found is kept and checked each frame; if it stops working it is looked for
// again, twice a second at most.
enum class Via { kUnknown, kItself, kCameraAt, kFrameAt };
Via g_via = Via::kUnknown;
uint32_t g_via_offset = 0;
double g_last_search = -1.0;
double g_last_reject_log = -100.0;
uint32_t g_rejects = 0;
bool g_dumped = false;
constexpr uint32_t kScanBytes = 0x200;

uint32_t Resolve(const uint8_t* base, uint32_t object, Via via, uint32_t offset) {
  switch (via) {
    case Via::kItself:
      return CameraFrame(base, object);
    case Via::kCameraAt:
    case Via::kFrameAt: {
      if (!Readable(object + offset, 4)) {
        return 0;
      }
      const uint32_t p = xerenge::LoadGuestU32(base, object + offset);
      if (p < 0x10000000u) {
        return 0;
      }
      if (via == Via::kCameraAt) {
        return CameraFrame(base, p);
      }
      return UsableFrame(base, p) ? p : 0;
    }
    default:
      return 0;
  }
}

const char* ViaName(Via via) {
  switch (via) {
    case Via::kItself:
      return "itself the RwCamera";
    case Via::kCameraAt:
      return "points to the RwCamera";
    case Via::kFrameAt:
      return "points to the camera's RwFrame";
    default:
      return "?";
  }
}

uint32_t FindFrame(const uint8_t* base, uint32_t object) {
  if (g_via != Via::kUnknown) {
    if (const uint32_t frame = Resolve(base, object, g_via, g_via_offset)) {
      return frame;
    }
  }
  const double now = Seconds();
  if (g_last_search >= 0.0 && now - g_last_search < 0.5) {
    return 0;
  }
  g_last_search = now;

  auto found = [&](Via via, uint32_t offset, uint32_t frame) {
    if (via != g_via || offset != g_via_offset) {
      REXLOG_INFO("telemetry: camera found - {:08X}{}{:X} {} (frame {:08X})", object,
                  via == Via::kItself ? "" : " +0x", via == Via::kItself ? 0u : offset,
                  ViaName(via), frame);
    }
    g_via = via;
    g_via_offset = offset;
    return frame;
  };

  if (const uint32_t frame = Resolve(base, object, Via::kItself, 0)) {
    return found(Via::kItself, 0, frame);
  }
  uint32_t scan = kScanBytes;
  while (scan > 16 && !Readable(object, scan)) {
    scan /= 2;
  }
  if (Readable(object, scan)) {
    for (const Via via : {Via::kCameraAt, Via::kFrameAt}) {
      for (uint32_t offset = 0; offset < scan; offset += 4) {
        if (const uint32_t frame = Resolve(base, object, via, offset)) {
          return found(via, offset, frame);
        }
      }
    }
  }

  ++g_rejects;
  if (!g_dumped && Readable(object, 64)) {
    g_dumped = true;
    std::string words;
    for (uint32_t i = 0; i < 16; ++i) {
      char text[12];
      std::snprintf(text, sizeof(text), "%s%08X", i ? " " : "",
                    xerenge::LoadGuestU32(base, object + i * 4));
      words += text;
    }
    REXLOG_WARN("telemetry: no camera found from {:08X} (scanned {} bytes); its first words: {}",
                object, scan, words);
  }
  if (now - g_last_reject_log >= 10.0) {
    g_last_reject_log = now;
    REXLOG_WARN("telemetry: still no race camera ({} tries); nothing is sent", g_rejects);
  }
  return 0;
}

// The race camera's matrix this frame, made into a car-like transform.
void RaceCamera(const uint8_t* base, uint32_t camera) {
  const Config& c = Settings();
  const uint32_t frame = FindFrame(base, camera);
  if (frame == 0) {
    return;
  }
  // The frame's LTM: up +96, at +112, pos +128 (right, +80, is not needed).
  const Vec3 cam_up = GuestVec(base, frame + 96);
  const Vec3 at = GuestVec(base, frame + 112);
  const Vec3 pos = GuestVec(base, frame + 128);
  if (!g_told_camera) {
    g_told_camera = true;
    REXLOG_INFO("telemetry: race camera {:08X}, frame {:08X}, viewport {:08X}", camera, frame,
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
