#pragma once

#include <cstdint>

// Motion telemetry: where the player is in a race, sent as UDP packets for a
// motion simulator (telemetry.cpp has the detail and the packet layout).
namespace xerenge::telemetry {

// CGtViewport::Update's view window is being made (widescreen.cpp's hook on
// sub_82353BA8): the viewport and its aspect, so the camera it sets next can
// be known for the race's.
void EnterViewport(uint32_t viewport, float aspect);
void LeaveViewport();

}  // namespace xerenge::telemetry
