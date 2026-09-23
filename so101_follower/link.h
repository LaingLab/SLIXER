// How the two boards and a PC talk to each other.
//
// The same small packets travel over two transports at once:
//   * ESP-NOW broadcast: board to board, no network needed, works wherever the radios reach.
//   * Wi-Fi UDP: once the boards join a network, anything on it can reach them, which is what lets a PC
//     drive the arm from another room.
// Duplicates are harmless because every packet carries a magic byte and the receiver just takes the newest.
// Addresses are learnt rather than configured: a board remembers whoever sends it a valid packet, and
// broadcasts once a second so a PC can find it without being told an address.
#pragma once

#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_now.h>
#include <esp_wifi.h>

#if __has_include("wifi_config.h")
#include "wifi_config.h"          // your network: a filled-in copy of wifi_config.example.h
#else
#include "wifi_config.example.h"  // none set up yet: the direct radio link only
#endif

namespace net {  // not "link": that clashes with the POSIX link() the core pulls in

constexpr uint16_t kPort = 50101;
constexpr uint8_t kFallbackChannel = 1;  // used only when Wi-Fi is not configured
constexpr uint32_t kBroadcastEveryMs = 1000;
constexpr uint32_t kPeerForgetMs = 10000;
constexpr int kMaxPeers = 4;

typedef void (*Handler)(const uint8_t* data, int len);

inline const uint8_t* broadcastMac() {
  static const uint8_t mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  return mac;
}

Handler g_handler = nullptr;
WiFiUDP g_udp;
bool g_udp_started = false;
uint32_t g_last_broadcast = 0;
IPAddress g_peer_ip[kMaxPeers];
uint32_t g_peer_seen[kMaxPeers] = {0};

inline void notePeer(const IPAddress& ip) {
  const uint32_t now = millis();
  int oldest = 0;
  for (int i = 0; i < kMaxPeers; i++) {
    if (g_peer_seen[i] != 0 && g_peer_ip[i] == ip) {
      g_peer_seen[i] = now;
      return;
    }
    if (g_peer_seen[i] < g_peer_seen[oldest]) oldest = i;
  }
  g_peer_ip[oldest] = ip;
  g_peer_seen[oldest] = now;
}

inline void onEspNow(const esp_now_recv_info_t*, const uint8_t* data, int len) {
  if (g_handler != nullptr) g_handler(data, len);
}

inline bool wifiUp() { return WiFi.status() == WL_CONNECTED; }

// Why the last attempt to join failed. `WiFi.status()` lumps most failures together as "disconnected",
// which is no use when the whole question is whether the board can't hear the network or isn't being let
// on to it. The radio knows; it says so in the disconnect event and nowhere else.
uint8_t g_last_reason = 0;
uint32_t g_attempts = 0;

inline const char* reasonText(uint8_t reason) {
  switch (reason) {
    case WIFI_REASON_NO_AP_FOUND: return "can't hear the network at all: antenna fitted? in range? 2.4 GHz?";
    case WIFI_REASON_AUTH_FAIL:
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT: return "wrong password";
    case WIFI_REASON_AUTH_EXPIRE: return "the network dropped us while joining: usually a weak signal";
    case WIFI_REASON_ASSOC_FAIL:
    case WIFI_REASON_ASSOC_EXPIRE: return "the network refused to associate: full, or filtering by MAC?";
    case WIFI_REASON_BEACON_TIMEOUT: return "lost sight of the network: signal too weak to hold";
    case WIFI_REASON_HANDSHAKE_TIMEOUT: return "the network stopped replying part way in: weak signal";
    default: return "";
  }
}

inline void onWifiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    g_last_reason = info.wifi_sta_disconnected.reason;
    g_attempts++;
  }
}

// Lists every network the board can hear, which settles the antenna question on its own: a board with no
// antenna hears nothing or almost nothing, wherever it is sitting. Takes a few seconds, during which the
// arm holds its last position rather than tracking the leader.
inline void scanNetworks() {
  Serial.println("[wifi] scanning, the arm will hold still for a few seconds...");
  const int found = WiFi.scanNetworks();
  if (found <= 0) {
    Serial.println("[wifi] nothing heard at all. A board with a working antenna always hears something.");
    return;
  }
  for (int i = 0; i < found; i++) {
    const bool ours = strcmp(WiFi.SSID(i).c_str(), kWifiSsid) == 0;
    Serial.printf("[wifi] %-28s %4d dBm  channel %-3d%s\n", WiFi.SSID(i).c_str(), (int)WiFi.RSSI(i),
                  (int)WiFi.channel(i), ours ? "   <-- the one we want" : "");
  }
  Serial.printf("[wifi] %d networks heard. Below about -80 dBm is too weak to join reliably.\n", found);
  WiFi.scanDelete();
}

inline bool begin(Handler on_packet) {
  g_handler = on_packet;
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // power saving would add tens of milliseconds to every packet
  esp_wifi_set_max_tx_power(84);
  // Keep the ordinary rates so an access point can still be used, and add Espressif's Long Range PHY,
  // which the board-to-board link uses for its much better reach.
  esp_wifi_set_protocol(WIFI_IF_STA,
                        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
  if (kWifiSsid[0] != '\0') {
    WiFi.onEvent(onWifiEvent);
    WiFi.setAutoReconnect(true);
    WiFi.begin(kWifiSsid, kWifiPassword);
  } else {
    esp_wifi_set_channel(kFallbackChannel, WIFI_SECOND_CHAN_NONE);
  }
  if (esp_now_init() != ESP_OK) return false;
  if (esp_now_register_recv_cb(onEspNow) != ESP_OK) return false;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastMac(), 6);
  peer.channel = 0;  // follow whatever channel we are on, which becomes the router's once connected
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) return false;
  esp_now_rate_config_t rate = {};
  rate.phymode = WIFI_PHY_MODE_LR;
  rate.rate = WIFI_PHY_RATE_LORA_250K;
  rate.ersu = false;
  rate.dcm = false;
  esp_now_set_peer_rate_config(broadcastMac(), &rate);
  return true;
}

// Call often: delivers anything that arrived over Wi-Fi to the same handler as the radio.
inline void poll() {
  if (wifiUp() && !g_udp_started) g_udp_started = g_udp.begin(kPort);
  if (!g_udp_started) return;
  for (int packet = g_udp.parsePacket(); packet > 0; packet = g_udp.parsePacket()) {
    uint8_t buffer[64];
    const int len = g_udp.read(buffer, sizeof buffer);
    if (len <= 0) continue;
    notePeer(g_udp.remoteIP());
    if (g_handler != nullptr) g_handler(buffer, len);
  }
}

inline void send(const void* data, size_t len) {
  esp_now_send(broadcastMac(), static_cast<const uint8_t*>(data), len);
  if (!g_udp_started || !wifiUp()) return;
  const uint32_t now = millis();
  for (int i = 0; i < kMaxPeers; i++) {
    if (g_peer_seen[i] == 0 || now - g_peer_seen[i] > kPeerForgetMs) continue;
    g_udp.beginPacket(g_peer_ip[i], kPort);
    g_udp.write(static_cast<const uint8_t*>(data), len);
    g_udp.endPacket();
  }
  if (now - g_last_broadcast >= kBroadcastEveryMs) {  // so a PC can discover us unprompted
    g_last_broadcast = now;
    g_udp.beginPacket(IPAddress(255, 255, 255, 255), kPort);
    g_udp.write(static_cast<const uint8_t*>(data), len);
    g_udp.endPacket();
  }
}

// Says what the Wi-Fi is doing, because "not connected" has very different causes: an unseen network
// usually means no antenna or too far away, while a refused one means the password is wrong.
inline const char* status() {
  static char text[160];
  if (kWifiSsid[0] == '\0') return "radio only (no network configured)";
  if (wifiUp()) {
    snprintf(text, sizeof text, "radio + wifi (%s, %d dBm)", WiFi.localIP().toString().c_str(), WiFi.RSSI());
    return text;
  }
  // The radio's own reason is the useful one whenever it has given us a real one; WiFi.status() only
  // gets a look in before the first attempt has come back.
  const char* why = reasonText(g_last_reason);
  if (why[0] == '\0') {
    switch (WiFi.status()) {
      case WL_NO_SSID_AVAIL: why = "can't see the network: antenna fitted? in range?"; break;
      case WL_CONNECT_FAILED: why = "network refused us: wrong password?"; break;
      case WL_IDLE_STATUS: why = "starting up"; break;
      default: why = "trying to connect"; break;
    }
    snprintf(text, sizeof text, "radio only (%s)", why);
  } else {
    snprintf(text, sizeof text, "radio only (%s; %lu tries)", why, (unsigned long)g_attempts);
  }
  return text;
}

}  // namespace net
