// Your network, so the boards can be reached from a PC anywhere it covers.
//
// Copy this file to wifi_config.h in the same folder and fill that copy in -- in both sketch folders, with
// the same values. wifi_config.h is kept out of git, so a password can't be committed by accident. Without
// it, or with the name left empty, the boards use only their direct radio link, which is all two arms
// need for teleoperation.
#pragma once

constexpr char kWifiSsid[] = "";      // e.g. "lab-wifi"
constexpr char kWifiPassword[] = "";  // e.g. "hunter2"
