/*
 * ══════════════════════════════════════════════════════════════════════════════
 *  GM MIDI Player  —  M5Stack Cardputer ADV
 *  PlatformIO / Arduino framework
 * ══════════════════════════════════════════════════════════════════════════════
 *
 *  Audio modes (interactive boot menu):
 *    1 -> ADV built-in ES8311       - 3.5mm jack + speaker, 22050Hz mono
 *    2 -> External I2S DAC          - MAX98357A/PCM5102,    22050Hz stereo
 *    3 -> PDM speaker GPIO2         - no extra hardware,    16000Hz mono
 *    4 -> PWM LEDC GPIO2            - fallback,             16000Hz mono
 *
 *  (-DCARDPUTER_V11 removes the ES8311 option and renumbers modes to 1-3)
 *
 *  Player controls:
 *    SPACE       Play / Pause
 *    ENTER / /   Next track
 *    ,           Previous track
 *    R           Restart current track
 *    L           Toggle loop
 *    + / -       Volume +3 / -3 dB
 *    M           Open the runtime menu (audio / SF2 / MIDI) without rebooting
 *
 *  Configuration stored on SD: /Midi/midi_player.cfg
 * ══════════════════════════════════════════════════════════════════════════════
 */

/*
 * English summary:
 * - Audio output is selected from an interactive boot menu.
 * - Player navigation uses ; . , / as up / down / left / right shortcuts.
 * - The config file is stored on the SD card at /Midi/midi_player.cfg.
 */

#include <Arduino.h>
#include <M5Cardputer.h>
#include <SD.h>
#include <SPI.h>
#include <driver/i2s.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <vector>
#include <algorithm>

// ── TinySoundFont ─────────────────────────────────────────────────────────────
#define TSF_NO_STDIO
#define TSF_IMPLEMENTATION
#define TSF_MALLOC(s)     heap_caps_malloc((s), MALLOC_CAP_DEFAULT)
#define TSF_REALLOC(p,s)  heap_caps_realloc((p),(s), MALLOC_CAP_DEFAULT)
#define TSF_FREE(p)       heap_caps_free(p)
#define TSF_MEMCPY(d,s,n) memcpy(d,s,n)
#define TSF_MEMSET(d,v,n) memset(d,v,n)
#include "../lib/TinySoundFont/tsf.h"

// ── TinyMidiLoader ────────────────────────────────────────────────────────────
#define TML_NO_STDIO
#define TML_IMPLEMENTATION
#define TML_ERROR(msg)     Serial.printf("[TML] ERROR: %s\r\n", msg)
#define TML_WARN(msg)      Serial.printf("[TML] WARN: %s\r\n", msg)
#define TML_MALLOC(s)     heap_caps_malloc((s), MALLOC_CAP_DEFAULT)
#define TML_REALLOC(p,s)  heap_caps_realloc((p),(s), MALLOC_CAP_DEFAULT)
#define TML_FREE(p)       heap_caps_free(p)
#include "../lib/TinySoundFont/tml.h"

// ── File browser + config ─────────────────────────────────────────────────────
#include "FileSelector.h"
#include "CardputerKeyboard.h"

// ══════════════════════════════════════════════════════════════════════════════
// PIN
// ══════════════════════════════════════════════════════════════════════════════

#define PIN_SD_CS    12
#define PIN_SD_SCK   40
#define PIN_SD_MISO  39
#define PIN_SD_MOSI  14

#define PIN_I2S_BCK   8     // External I2S DAC (mode 2)
#define PIN_I2S_WS    7
#define PIN_I2S_DOUT  6
#define PIN_SPEAKER   2     // Internal speaker (mode 3 / 4)
#define PIN_PDM_CLK   41    // Internal PDM clock (leave unconnected)

#define LEDC_CH    0
#define LEDC_FREQ  312500
#define LEDC_BITS  8

// ══════════════════════════════════════════════════════════════════════════════
// AUDIO
// ══════════════════════════════════════════════════════════════════════════════

enum AudioMode {
  AUDIO_NONE    = 0,
  AUDIO_ES8311  = 1,
  AUDIO_I2S_DAC = 2,
  AUDIO_PDM     = 3,
  AUDIO_PWM     = 4
};

static AudioMode g_audioMode  = AUDIO_NONE;
static uint32_t  g_sampleRate = 22050;
static i2s_port_t g_i2sPort   = I2S_NUM_0;
static bool       g_audioReady = false;

#define CHUNK_FRAMES  256
#define ES8311_CHUNK_FRAMES 1024
#define I2S_DMA_BUFS  6
#define I2S_DMA_LEN   256
#define MAX_VOICES    16
static constexpr uint32_t UI_TASK_STACK = 12288;
static constexpr uint32_t AUDIO_TASK_STACK = 8192;
static constexpr UBaseType_t AUDIO_TASK_PRIORITY = 3;
static constexpr size_t ES8311_QUEUE_BUFFERS = 3;
static constexpr uint32_t MIDI_RAM_HEADROOM = 96 * 1024;
static constexpr uint32_t IDLE_SPLASH_DELAY_MS = 5000;
enum : uint8_t {
  ES8311_BUF_EMPTY  = 0,
  ES8311_BUF_READY  = 1,
  ES8311_BUF_QUEUED = 2,
};

static int16_t g_audioBuf[CHUNK_FRAMES];
static int16_t g_i2sBuf [CHUNK_FRAMES * 2];
static int16_t g_es8311Buf[ES8311_QUEUE_BUFFERS][ES8311_CHUNK_FRAMES];
static uint8_t g_es8311BufState[ES8311_QUEUE_BUFFERS] = {};
static uint8_t g_es8311QueueOrder[ES8311_QUEUE_BUFFERS] = {};
static size_t  g_es8311QueueHead  = 0;
static size_t  g_es8311QueueCount = 0;

extern const uint8_t kBootSplashJpgStart[] asm("_binary_media_ScreenTitle_boot_jpg_start");
extern const uint8_t kBootSplashJpgEnd[]   asm("_binary_media_ScreenTitle_boot_jpg_end");

static volatile uint32_t g_pwmIdx  = 0;
static volatile bool     g_pwmDone = false;
static hw_timer_t*       g_pwmTimer = nullptr;
static SPIClass          g_spiSD(HSPI);

static bool loadMidi(int idx);
static void tickMidi(double deltaMs);
static void openFileSelector();
static void redrawFull();
static void redrawPartial();
static void playerUiStep();
static void setPlaybackState(bool playing);
static void showBootSplashImage(uint32_t holdMs);
static void shutdownAudio();
static void saveCurrentMidiConfig(int idx);

static inline bool useStereoSynthOutput() {
  return g_audioMode == AUDIO_I2S_DAC;
}

static inline enum TSFOutputMode currentSynthOutputMode() {
  return useStereoSynthOutput() ? TSF_STEREO_INTERLEAVED : TSF_MONO;
}

static void resetEs8311Buffers(bool stopSpeaker = false) {
  memset(g_es8311BufState, ES8311_BUF_EMPTY, sizeof(g_es8311BufState));
  memset(g_es8311QueueOrder, 0, sizeof(g_es8311QueueOrder));
  g_es8311QueueHead = 0;
  g_es8311QueueCount = 0;
  if (stopSpeaker && g_audioReady && g_audioMode == AUDIO_ES8311) {
    M5Cardputer.Speaker.stop(0);
  }
}

static size_t es8311SpeakerQueuedCount(void) {
  size_t count = M5Cardputer.Speaker.isPlaying(0);
  return (count > 2) ? 2 : count;
}

static void es8311ReclaimConsumedBuffers(void) {
  size_t active = es8311SpeakerQueuedCount();
  while (g_es8311QueueCount > active) {
    uint8_t idx = g_es8311QueueOrder[g_es8311QueueHead];
    if (idx < ES8311_QUEUE_BUFFERS) {
      g_es8311BufState[idx] = ES8311_BUF_EMPTY;
    }
    g_es8311QueueHead = (g_es8311QueueHead + 1) % ES8311_QUEUE_BUFFERS;
    --g_es8311QueueCount;
  }
}

static int es8311FindBuffer(uint8_t state) {
  for (int i = 0; i < (int)ES8311_QUEUE_BUFFERS; ++i) {
    if (g_es8311BufState[i] == state) return i;
  }
  return -1;
}

void IRAM_ATTR pwmISR() {
  if (g_pwmIdx < (uint32_t)CHUNK_FRAMES)
    ledcWrite(LEDC_CH, (uint8_t)((g_audioBuf[g_pwmIdx++] >> 8) + 128));
  else { g_pwmDone = true; g_pwmIdx = 0; }
}

// ══════════════════════════════════════════════════════════════════════════════
// GM INSTRUMENT NAMES
// ══════════════════════════════════════════════════════════════════════════════

static const char* const GM_NAMES[128] = {
  "AcGrand","BrGrand","ElGrand","HnkyTnk","ElPno1","ElPno2","Harpsi","Clavi",
  "Celesta","Glocksp","MusicBx","Vibraph","Marimba","Xyloph","TubBell","Dulcim",
  "DrawOrg","PercOrg","RockOrg","ChrOrg", "ReedOrg","Accord","Harmon","TangAcc",
  "AcGtr",  "AcBsGtr","ElJzGtr","ElClGtr","MtElGtr","OvElGtr","DstGtr","GtrHrm",
  "AcBass", "FinElBs","PckElBs","FrtlBs", "SlpBass","SlpBs2","SynBs1","SynBs2",
  "Violin", "Viola",  "Cello",  "Contrbs","TrmStr","PzzStr","OrchHit","Timpani",
  "StrEns1","StrEns2","SynStr1","SynStr2","ChoAah","VoiceOoh","SynVoice","OrcHit",
  "Trumpet","Trombon","Tuba",   "MtTrump","FrnHorn","BrassSec","SynBrs1","SynBrs2",
  "SopSax", "AltSax", "TenrSax","BariSax","Oboe",  "EnglHrn","Bassoon","Clrnet",
  "Piccolo","Flute",  "Recorder","PanFlut","BlwnBot","Shakuhc","Whistle","Ocarina",
  "SqWave", "SawWave","CallOrg","ChiffLd","CharSyn","VoiceLd","FifthLd","Bass+Ld",
  "NewAgePd","WarmPad","PolySyn","ChoirPd","BowedGl","MetalPd","HaloPad","SweepPd",
  "Rain",   "Soundtrk","Crystal","Atmosphr","Bright","Goblins","Echoes","Sci-Fi",
  "Sitar",  "Banjo",  "Shamis", "Koto",   "Kalimba","BagPipe","Fiddle","Shanai",
  "TinklBl","Agogo",  "SteelDr","WoodBlk","TaikoDr","MeldTom","SynDrum","RevCymb",
  "GtrFret","BrthNois","Seashore","BirdTwt","Teleph","Heli",  "Applaus","Gunshot"
};

// ══════════════════════════════════════════════════════════════════════════════
// CHANNEL STATE
// ══════════════════════════════════════════════════════════════════════════════

struct ChannelInfo {
  bool     active    = false;
  uint8_t  program   = 0;
  uint8_t  note      = 0;
  uint8_t  velocity  = 0;
  uint8_t  volume    = 100;
  bool     isDrum    = false;
  uint32_t lastNoteMs = 0;
  uint8_t  noteCount = 0;
  uint8_t  noteHistory[4] = {};
  uint8_t  histIdx   = 0;
};

struct MidiTrackDescriptor {
  uint32_t start = 0;
  uint32_t end   = 0;
};

struct MidiEvent {
  uint32_t tick   = 0;
  uint8_t  type   = 0;
  uint8_t  channel = 0;
  uint8_t  a      = 0;
  uint8_t  b      = 0;
  uint8_t  tempo[3] = { 0, 0, 0 };
};

struct MidiTrackState {
  uint32_t start = 0;
  uint32_t end   = 0;
  uint32_t pos   = 0;
  uint32_t tick  = 0;
  uint8_t  runningStatus = 0;
  bool     hasEvent = false;
  bool     finished = false;
  MidiEvent event;
};

struct Es8311RenderStats {
  uint32_t eventUs      = 0;
  uint32_t synthUs      = 0;
  uint32_t midiEvents   = 0;
  uint32_t synthSegments = 0;
};

struct PerfWindow {
  uint32_t startedMs       = 0;
  uint32_t uiCalls         = 0;
  uint64_t uiUsTotal       = 0;
  uint32_t uiUsMax         = 0;
  uint32_t chunkCalls      = 0;
  uint64_t chunkUsTotal    = 0;
  uint32_t chunkUsMax      = 0;
  uint32_t lateChunks      = 0;
  uint64_t renderUsTotal   = 0;
  uint32_t renderUsMax     = 0;
  uint64_t eventUsTotal    = 0;
  uint32_t eventUsMax      = 0;
  uint64_t synthUsTotal    = 0;
  uint32_t synthUsMax      = 0;
  uint32_t midiEvents      = 0;
  uint32_t synthSegments   = 0;
  uint32_t playCalls       = 0;
  uint64_t playUsTotal     = 0;
  uint32_t playUsMax       = 0;
  uint32_t playOk          = 0;
  uint32_t playFail        = 0;
};

// ══════════════════════════════════════════════════════════════════════════════
// GLOBALS
// ══════════════════════════════════════════════════════════════════════════════

static tsf*              g_tsf      = nullptr;
static SemaphoreHandle_t g_tsfMutex = nullptr;
static tml_message*      g_midiRoot = nullptr;
static tml_message*      g_midiCur  = nullptr;

static std::vector<MidiTrackState> g_midiTracks;
static bool         g_midiLoaded   = false;
static uint16_t     g_midiDivision = 0;
static uint32_t     g_midiTempoUs  = 500000;
static double       g_midiTick     = 0.0;
static double       g_midiMs       = 0.0;

static std::vector<String> g_midiList;
static int          g_midiIdx  = 0;
static bool         g_playing  = false;
static bool         g_looping  = false;
static float        g_vol_dB   = 0.0f;

static ChannelInfo  g_ch[16];
static bool         g_fullRedraw = true;
static bool         g_needRedraw = true;
static uint32_t     g_lastUiMs   = 0;
static uint32_t     g_lastTickUs = 0;
static uint32_t     g_songDurMs  = 0;
static uint16_t     g_dirtyChannelMask = 0xFFFFu;
static uint32_t     g_lastHeaderElapsedSec = UINT32_MAX;
static uint32_t     g_lastHeaderTotalSec   = UINT32_MAX;
static int          g_lastHeaderMidiIdx    = -1;
static bool         g_lastHeaderPlaying    = false;
static bool         g_lastHeaderLooping    = false;
static int          g_lastProgressW        = -1;
static uint32_t     g_lastHeaderTitleScroll = UINT32_MAX;
static bool         g_trackAdvancePending  = false;
static uint32_t     g_trackAdvanceAtMs     = 0;
static uint32_t     g_lastPlaybackActivityMs = 0;
static bool         g_idleSplashActive = false;
static PerfWindow   g_perf;

static PlayerConfig g_cfg;
static File         g_sf2File;
static File         g_midiFile;
static uint8_t*     g_midiData = nullptr;
static size_t       g_midiDataSize = 0;
static bool         g_midiFromRam = false;

static void perfReset(uint32_t nowMs) {
  memset(&g_perf, 0, sizeof(g_perf));
  g_perf.startedMs = nowMs;
}

static bool keysStateHasAnyInput(const cardputer_keyboard::KeysState& ks) {
  return ks.tab || ks.fn || ks.shift || ks.ctrl || ks.opt || ks.alt
      || ks.del || ks.enter || ks.space
      || !ks.word.empty()
      || !ks.hid_keys.empty()
      || !ks.modifier_keys.empty();
}

static void perfMaybeLog(void) {
  uint32_t now = millis();
  if (g_perf.startedMs == 0) {
    perfReset(now);
    return;
  }

  uint32_t elapsedMs = now - g_perf.startedMs;
  if (elapsedMs < 1000) return;

  uint32_t chunkBudgetUs = 0;
  if (g_audioMode == AUDIO_ES8311 && g_sampleRate > 0) {
    chunkBudgetUs = (uint32_t)((1000000ULL * ES8311_CHUNK_FRAMES) / g_sampleRate);
  }

  uint32_t uiAvgUs     = g_perf.uiCalls    ? (uint32_t)(g_perf.uiUsTotal / g_perf.uiCalls)       : 0;
  uint32_t chunkAvgUs  = g_perf.chunkCalls ? (uint32_t)(g_perf.chunkUsTotal / g_perf.chunkCalls) : 0;
  uint32_t renderAvgUs = g_perf.chunkCalls ? (uint32_t)(g_perf.renderUsTotal / g_perf.chunkCalls) : 0;
  uint32_t eventAvgUs  = g_perf.chunkCalls ? (uint32_t)(g_perf.eventUsTotal / g_perf.chunkCalls)  : 0;
  uint32_t synthAvgUs  = g_perf.chunkCalls ? (uint32_t)(g_perf.synthUsTotal / g_perf.chunkCalls)  : 0;
  uint32_t playAvgUs   = g_perf.playCalls  ? (uint32_t)(g_perf.playUsTotal / g_perf.playCalls)     : 0;

  Serial.printf(
      "[PERF] ui=%lu/%luus chunk=%lu/%luus late=%lu/%lu budget=%luus "
      "evt=%lu/%luus synth=%lu/%luus seg=%lu ev=%lu play=%lu/%luus ok=%lu fail=%lu\r\n",
      (unsigned long)uiAvgUs,
      (unsigned long)g_perf.uiUsMax,
      (unsigned long)chunkAvgUs,
      (unsigned long)g_perf.chunkUsMax,
      (unsigned long)g_perf.lateChunks,
      (unsigned long)g_perf.chunkCalls,
      (unsigned long)chunkBudgetUs,
      (unsigned long)eventAvgUs,
      (unsigned long)g_perf.eventUsMax,
      (unsigned long)synthAvgUs,
      (unsigned long)g_perf.synthUsMax,
      (unsigned long)g_perf.synthSegments,
      (unsigned long)g_perf.midiEvents,
      (unsigned long)playAvgUs,
      (unsigned long)g_perf.playUsMax,
      (unsigned long)g_perf.playOk,
      (unsigned long)g_perf.playFail);

  perfReset(now);
}

static inline void markChannelDirty(int ch) {
  if ((unsigned)ch < 16u) {
    g_dirtyChannelMask |= (uint16_t)(1u << ch);
    g_needRedraw = true;
  }
}

static inline void markAllChannelsDirty(void) {
  g_dirtyChannelMask = 0xFFFFu;
  g_needRedraw = true;
}

static inline int currentProgressWidth(void) {
  if (g_songDurMs > 0 && g_midiMs > 0.0) {
    int w = (int)(240.0 * g_midiMs / (double)g_songDurMs);
    return (w > 240) ? 240 : w;
  }
  return 0;
}

static String currentTrackTitle(void) {
  if (g_midiList.empty()) return "";

  String fname = g_midiList[g_midiIdx];
  int sl = fname.lastIndexOf('/');
  if (sl >= 0) fname = fname.substring(sl + 1);
  int dt = fname.lastIndexOf('.');
  if (dt > 0) fname = fname.substring(0, dt);
  return fname;
}

static void buildHeaderInfoText(char* info, size_t infoSize) {
  uint32_t el = (uint32_t)(g_midiMs / 1000.0);
  if (g_songDurMs > 0) {
    uint32_t tot = g_songDurMs / 1000;
    snprintf(info, infoSize, "%2d/%d %u:%02u/%u:%02u",
             g_midiIdx + 1, (int)g_midiList.size(),
             el / 60, el % 60, tot / 60, tot % 60);
  } else {
    snprintf(info, infoSize, "%2d/%d %u:%02u/--:--",
             g_midiIdx + 1, (int)g_midiList.size(),
             el / 60, el % 60);
  }
}

static uint32_t currentHeaderTitleScrollIndex(void) {
  String title = currentTrackTitle();
  if (title.isEmpty()) return 0;

  char info[26];
  buildHeaderInfoText(info, sizeof(info));
  int infoX = 240 - (int)strlen(info) * 6 - 2;
  int titleX = 38;
  int titleW = infoX - titleX - 4;
  int visibleChars = titleW / 6;
  if (visibleChars <= 0 || (int)title.length() <= visibleChars) return 0;

  return (millis() / 250) % (uint32_t)(title.length() + 3);
}

static String currentHeaderTitleWindow(void) {
  String title = currentTrackTitle();
  if (title.isEmpty()) return title;

  char info[26];
  buildHeaderInfoText(info, sizeof(info));
  int infoX = 240 - (int)strlen(info) * 6 - 2;
  int titleX = 38;
  int titleW = infoX - titleX - 4;
  int visibleChars = titleW / 6;
  if (visibleChars <= 0) return "";
  if ((int)title.length() <= visibleChars) return title;

  String marquee = title + "   " + title + "   ";
  uint32_t scroll = currentHeaderTitleScrollIndex();
  int start = (int)scroll;
  int end = start + visibleChars;
  if (end > (int)marquee.length()) end = marquee.length();
  String out = marquee.substring(start, end);
  while ((int)out.length() < visibleChars) out += ' ';
  return out;
}

static void syncUiCache(void) {
  g_lastHeaderElapsedSec = (uint32_t)(g_midiMs / 1000.0);
  g_lastHeaderTotalSec   = g_songDurMs / 1000;
  g_lastHeaderMidiIdx    = g_midiIdx;
  g_lastHeaderPlaying    = g_playing;
  g_lastHeaderLooping    = g_looping;
  g_lastProgressW        = currentProgressWidth();
  g_lastHeaderTitleScroll = currentHeaderTitleScrollIndex();
}

static void invalidateUiCache(void) {
  g_lastHeaderElapsedSec = UINT32_MAX;
  g_lastHeaderTotalSec   = UINT32_MAX;
  g_lastHeaderMidiIdx    = -1;
  g_lastHeaderPlaying    = !g_playing;
  g_lastHeaderLooping    = !g_looping;
  g_lastProgressW        = -1;
  g_lastHeaderTitleScroll = UINT32_MAX;
  markAllChannelsDirty();
}

static void queueNextTrackAdvance(void) {
  if (!g_trackAdvancePending) {
    g_trackAdvancePending = true;
    g_trackAdvanceAtMs = millis() + 600;
  }
}

static void clearTrackAdvance(void) {
  g_trackAdvancePending = false;
  g_trackAdvanceAtMs = 0;
}

// ══════════════════════════════════════════════════════════════════════════════
// SD STREAM ADAPTERS
// ══════════════════════════════════════════════════════════════════════════════

static int sf2_read(void*, void* p, unsigned s) { return (int)g_sf2File.read((uint8_t*)p, s); }
static int sf2_skip(void*, unsigned n)          { return g_sf2File.seek(g_sf2File.position() + n) ? 1 : 0; }

static int tml_read(void*, void* p, unsigned s) { return (int)g_midiFile.read((uint8_t*)p, s); }

static bool midiHasValidHeader(const uint8_t* data, size_t size) {
  return size >= 14
      && data[0] == 'M' && data[1] == 'T' && data[2] == 'h' && data[3] == 'd'
      && data[7] == 6
      && data[9] <= 2;
}

static uint16_t midiReadBE16(const uint8_t* p) {
  return (uint16_t)((p[0] << 8) | p[1]);
}

static uint32_t midiReadBE32(const uint8_t* p) {
  return ((uint32_t)p[0] << 24)
       | ((uint32_t)p[1] << 16)
       | ((uint32_t)p[2] << 8)
       | (uint32_t)p[3];
}

static uint32_t midiTempoValue(const MidiEvent& evt) {
  return ((uint32_t)evt.tempo[0] << 16)
       | ((uint32_t)evt.tempo[1] << 8)
       | (uint32_t)evt.tempo[2];
}

static inline bool midiSourceReady(void) {
  return (g_midiData != nullptr && g_midiDataSize > 0) || (bool)g_midiFile;
}

static inline uint32_t midiSourceSize(void) {
  if (g_midiData != nullptr && g_midiDataSize > 0) return (uint32_t)g_midiDataSize;
  return g_midiFile ? (uint32_t)g_midiFile.size() : 0;
}

static void midiReleaseRamBuffer(void) {
  if (g_midiData) {
    heap_caps_free(g_midiData);
    g_midiData = nullptr;
  }
  g_midiDataSize = 0;
  g_midiFromRam = false;
}

static bool midiFileSeek(uint32_t pos) {
  if (g_midiData != nullptr && g_midiDataSize > 0) {
    return pos <= g_midiDataSize;
  }
  return ((uint32_t)g_midiFile.position() == pos) ? true : g_midiFile.seek(pos);
}

static bool midiReadBytes(uint32_t& pos, uint32_t end, uint8_t* dst, size_t len) {
  if ((uint64_t)pos + len > end) return false;
  if (g_midiData != nullptr && g_midiDataSize > 0) {
    memcpy(dst, g_midiData + pos, len);
    pos += (uint32_t)len;
    return true;
  }
  if (!midiFileSeek(pos)) return false;
  size_t n = g_midiFile.read(dst, len);
  if (n != len) return false;
  pos = (uint32_t)g_midiFile.position();
  return true;
}

static int midiReadByte(uint32_t& pos, uint32_t end) {
  uint8_t b = 0;
  return midiReadBytes(pos, end, &b, 1) ? b : -1;
}

static bool midiReadVarLen(uint32_t& pos, uint32_t end, uint32_t& out) {
  out = 0;
  for (int i = 0; i < 4; ++i) {
    int c = midiReadByte(pos, end);
    if (c < 0) return false;
    if (c & 0x80) out = ((out | (uint32_t)(c & 0x7F)) << 7);
    else {
      out |= (uint32_t)c;
      return true;
    }
  }
  return false;
}

static bool midiHasPendingEvents(const std::vector<MidiTrackState>& tracks) {
  for (const auto& tr : tracks) {
    if (tr.hasEvent) return true;
  }
  return false;
}

static bool midiHasPendingEvents() {
  return midiHasPendingEvents(g_midiTracks);
}

static int midiFindNextTrack(const std::vector<MidiTrackState>& tracks) {
  int best = -1;
  uint32_t bestTick = 0;
  for (int i = 0; i < (int)tracks.size(); ++i) {
    if (!tracks[i].hasEvent) continue;
    if (best < 0 || tracks[i].event.tick < bestTick) {
      best = i;
      bestTick = tracks[i].event.tick;
    }
  }
  return best;
}

static void midiCloseStream() {
  g_midiTracks.clear();
  g_midiLoaded = false;
  g_midiDivision = 0;
  g_midiTempoUs = 500000;
  g_midiTick = 0.0;
  g_midiMs = 0.0;
  g_songDurMs = 0;
  if (g_midiFile) g_midiFile.close();
  midiReleaseRamBuffer();
  resetEs8311Buffers(true);
}

static bool midiReadDescriptors(std::vector<MidiTrackDescriptor>& tracks,
                                uint16_t& format,
                                uint16_t& division) {
  tracks.clear();
  if (!midiSourceReady()) return false;

  uint32_t fileEnd = midiSourceSize();
  uint32_t pos = 0;
  uint8_t hdr[14];
  if (!midiReadBytes(pos, fileEnd, hdr, sizeof(hdr)) || !midiHasValidHeader(hdr, sizeof(hdr))) {
    Serial.println("[MIDI] Invalid MThd header");
    return false;
  }

  format = midiReadBE16(&hdr[8]);
  uint16_t numTracks = midiReadBE16(&hdr[10]);
  division = midiReadBE16(&hdr[12]);
  if ((division & 0x8000u) || division == 0 || numTracks == 0 || format > 2) {
    Serial.println("[MIDI] Unsupported format or timing");
    return false;
  }

  tracks.reserve(numTracks);
  for (uint16_t i = 0; i < numTracks; ++i) {
    uint8_t trk[8];
    if (!midiReadBytes(pos, fileEnd, trk, sizeof(trk))) {
      Serial.println("[MIDI] Unexpected EOF while reading MTrk");
      return false;
    }
    if (trk[0] != 'M' || trk[1] != 'T' || trk[2] != 'r' || trk[3] != 'k') {
      Serial.println("[MIDI] Invalid MTrk header");
      return false;
    }
    uint32_t len = midiReadBE32(&trk[4]);
    if ((uint64_t)pos + len > fileEnd) {
      Serial.println("[MIDI] Track length exceeds file size");
      return false;
    }
    MidiTrackDescriptor desc;
    desc.start = pos;
    desc.end = pos + len;
    tracks.push_back(desc);
    pos += len;
  }

  return true;
}

static bool midiTrackReadNextEvent(MidiTrackState& tr) {
  tr.hasEvent = false;
  while (!tr.finished && tr.pos < tr.end) {
    uint32_t delta = 0;
    if (!midiReadVarLen(tr.pos, tr.end, delta)) break;
    tr.tick += delta;

    int status = midiReadByte(tr.pos, tr.end);
    if (status < 0) break;
    if ((status & 0x80) == 0) {
      if ((tr.runningStatus & 0x80) == 0) break;
      tr.pos--;
      status = tr.runningStatus;
    } else if (status < 0xF0) {
      tr.runningStatus = (uint8_t)status;
    }

    MidiEvent evt{};
    evt.tick = tr.tick;
    evt.type = 0;
    evt.channel = (uint8_t)(status & 0x0F);

    if (status == 0xFF) {
      int metaType = midiReadByte(tr.pos, tr.end);
      uint32_t len = 0;
      if (metaType < 0 || !midiReadVarLen(tr.pos, tr.end, len)) break;
      if (metaType == TML_EOT) {
        if ((uint64_t)tr.pos + len > tr.end) break;
        tr.pos += len;
        evt.type = TML_EOT;
        tr.event = evt;
        tr.hasEvent = true;
        return true;
      }
      if (metaType == TML_SET_TEMPO && len == 3) {
        if (!midiReadBytes(tr.pos, tr.end, evt.tempo, 3)) break;
        evt.type = TML_SET_TEMPO;
        tr.event = evt;
        tr.hasEvent = true;
        return true;
      }
      if ((uint64_t)tr.pos + len > tr.end) break;
      tr.pos += len;
      continue;
    }

    if (status == TML_SYSEX || status == TML_EOX) {
      uint32_t len = 0;
      if (!midiReadVarLen(tr.pos, tr.end, len)) break;
      if ((uint64_t)tr.pos + len > tr.end) break;
      tr.pos += len;
      continue;
    }

    uint8_t hi = (uint8_t)(status & 0xF0);
    switch (hi) {
      case TML_NOTE_OFF:
      case TML_NOTE_ON:
      case TML_KEY_PRESSURE:
      case TML_CONTROL_CHANGE: {
        int a = midiReadByte(tr.pos, tr.end);
        int b = midiReadByte(tr.pos, tr.end);
        if (a < 0 || b < 0) {
          tr.finished = true;
          tr.hasEvent = false;
          return false;
        }
        evt.type = hi;
        evt.a = (uint8_t)(a & 0x7F);
        evt.b = (uint8_t)(b & 0x7F);
        tr.event = evt;
        tr.hasEvent = true;
        return true;
      }
      case TML_PITCH_BEND: {
        int lsb = midiReadByte(tr.pos, tr.end);
        int msb = midiReadByte(tr.pos, tr.end);
        if (lsb < 0 || msb < 0) {
          tr.finished = true;
          tr.hasEvent = false;
          return false;
        }
        evt.type = hi;
        evt.a = (uint8_t)(lsb & 0x7F);
        evt.b = (uint8_t)(msb & 0x7F);
        tr.event = evt;
        tr.hasEvent = true;
        return true;
      }
      case TML_PROGRAM_CHANGE:
      case TML_CHANNEL_PRESSURE: {
        int a = midiReadByte(tr.pos, tr.end);
        if (a < 0) {
          tr.finished = true;
          tr.hasEvent = false;
          return false;
        }
        evt.type = hi;
        evt.a = (uint8_t)(a & 0x7F);
        tr.event = evt;
        tr.hasEvent = true;
        return true;
      }
      default:
        tr.finished = true;
        return false;
    }
  }

  tr.finished = true;
  tr.hasEvent = false;
  return false;
}

static void midiInitTrackState(MidiTrackState& tr, const MidiTrackDescriptor& desc) {
  tr = MidiTrackState{};
  tr.start = desc.start;
  tr.end   = desc.end;
  tr.pos   = desc.start;
  midiTrackReadNextEvent(tr);
}

static void midiPrimeTracks(const std::vector<MidiTrackDescriptor>& descs,
                            std::vector<MidiTrackState>& tracks) {
  tracks.clear();
  tracks.reserve(descs.size());
  for (const auto& desc : descs) {
    MidiTrackState tr;
    midiInitTrackState(tr, desc);
    tracks.push_back(tr);
  }
}

static bool midiScanDuration(const std::vector<MidiTrackDescriptor>& descs,
                             uint16_t division,
                             uint32_t& outDurationMs,
                             uint32_t& outFirstEventMs) {
  outDurationMs = 0;
  outFirstEventMs = 0;
  if (division == 0) return false;

  std::vector<MidiTrackState> tracks;
  midiPrimeTracks(descs, tracks);
  if (!midiHasPendingEvents(tracks)) return true;

  uint32_t tempoUs = 500000;
  uint32_t currentTick = 0;
  double currentMs = 0.0;
  bool firstSet = false;

  while (true) {
    int best = midiFindNextTrack(tracks);
    if (best < 0) break;
    uint32_t nextTick = tracks[best].event.tick;
    if (nextTick > currentTick) {
      currentMs += (double)(nextTick - currentTick) * tempoUs / (1000.0 * division);
      currentTick = nextTick;
    }
    if (!firstSet) {
      outFirstEventMs = (uint32_t)(currentMs + 0.5);
      firstSet = true;
    }

    for (auto& tr : tracks) {
      if (!tr.hasEvent || tr.event.tick != currentTick) continue;
      if (tr.event.type == TML_SET_TEMPO) {
        uint32_t newTempo = midiTempoValue(tr.event);
        if (newTempo > 0) tempoUs = newTempo;
      }
      midiTrackReadNextEvent(tr);
    }
  }

  outDurationMs = (uint32_t)(currentMs + 0.5);
  return true;
}

static void midiProcessEvent(const MidiEvent& evt) {
  int ch = evt.channel;
  if (ch < 0 || ch > 15) return;

  switch (evt.type) {
    case TML_PROGRAM_CHANGE:
      g_ch[ch].program = evt.a;
      tsf_channel_set_presetnumber(g_tsf, ch, evt.a, ch == 9);
      markChannelDirty(ch);
      break;

    case TML_NOTE_ON:
      if (evt.b > 0) {
        g_ch[ch].note       = evt.a;
        g_ch[ch].velocity   = evt.b;
        g_ch[ch].active     = true;
        g_ch[ch].lastNoteMs = millis();
        if (g_ch[ch].noteCount < 255) g_ch[ch].noteCount++;
        g_ch[ch].noteHistory[g_ch[ch].histIdx & 3] = evt.a;
        g_ch[ch].histIdx++;
        tsf_channel_note_on(g_tsf, ch, evt.a, evt.b / 127.0f);
      } else {
        if (g_ch[ch].noteCount > 0) g_ch[ch].noteCount--;
        if (!g_ch[ch].noteCount) g_ch[ch].active = false;
        tsf_channel_note_off(g_tsf, ch, evt.a);
      }
      markChannelDirty(ch);
      break;

    case TML_NOTE_OFF:
      if (g_ch[ch].noteCount > 0) g_ch[ch].noteCount--;
      if (!g_ch[ch].noteCount) g_ch[ch].active = false;
      tsf_channel_note_off(g_tsf, ch, evt.a);
      markChannelDirty(ch);
      break;

    case TML_PITCH_BEND:
      tsf_channel_set_pitchwheel(g_tsf, ch, (uint16_t)(((uint16_t)evt.b << 7) | evt.a));
      break;

    case TML_CONTROL_CHANGE:
      tsf_channel_midi_control(g_tsf, ch, evt.a, evt.b);
      if (evt.a == 7) {
        g_ch[ch].volume = evt.b;
        markChannelDirty(ch);
      }
      break;

    case TML_SET_TEMPO: {
      uint32_t newTempo = midiTempoValue(evt);
      if (newTempo > 0) g_midiTempoUs = newTempo;
      break;
    }

    default:
      break;
  }
}

static void midiDecayVisuals(void) {
  uint32_t decayNow = millis();
  for (int i = 0; i < 16; i++) {
    if (g_ch[i].active && (decayNow - g_ch[i].lastNoteMs) > 600) {
      g_ch[i].active    = false;
      g_ch[i].noteCount = 0;
      markChannelDirty(i);
    }
  }
}

static uint32_t midiProcessDueEventsLocked(void) {
  uint32_t processed = 0;
  while (midiHasPendingEvents()) {
    int best = midiFindNextTrack(g_midiTracks);
    if (best < 0) break;
    uint32_t nextTick = g_midiTracks[best].event.tick;
    if ((double)nextTick > g_midiTick + 1e-9) break;
    g_midiTick = (double)nextTick;
    for (auto& tr : g_midiTracks) {
      if (!tr.hasEvent || tr.event.tick != nextTick) continue;
      midiProcessEvent(tr.event);
      ++processed;
      midiTrackReadNextEvent(tr);
    }
  }
  return processed;
}

static Es8311RenderStats renderEs8311Chunk(int16_t* buf, size_t frames) {
  Es8311RenderStats stats{};
  if (!g_tsf || !g_playing || !g_midiLoaded || g_midiDivision == 0) {
    memset(buf, 0, frames * sizeof(int16_t));
    return stats;
  }

  size_t offset = 0;
  xSemaphoreTake(g_tsfMutex, portMAX_DELAY);
  while (offset < frames) {
    uint32_t eventStartUs = micros();
    stats.midiEvents += midiProcessDueEventsLocked();
    stats.eventUs += micros() - eventStartUs;

    if (!midiHasPendingEvents()) {
      size_t remain = frames - offset;
      uint32_t synthStartUs = micros();
      tsf_render_short(g_tsf, buf + offset, (int)remain, 0);
      stats.synthUs += micros() - synthStartUs;
      ++stats.synthSegments;
      double ms = remain * 1000.0 / g_sampleRate;
      g_midiMs += ms;
      g_midiTick += ms * (1000.0 * g_midiDivision) / g_midiTempoUs;
      offset = frames;
      break;
    }

    int best = midiFindNextTrack(g_midiTracks);
    if (best < 0) break;

    uint32_t nextTick = g_midiTracks[best].event.tick;
    double msToNext = ((double)nextTick - g_midiTick) * g_midiTempoUs
                    / (1000.0 * g_midiDivision);
    if (msToNext <= 0.0) continue;

    size_t framesLeft = frames - offset;
    size_t framesSeg = (size_t)((msToNext * g_sampleRate) / 1000.0);
    if (framesSeg == 0) framesSeg = 1;
    if (framesSeg > framesLeft) framesSeg = framesLeft;

    uint32_t synthStartUs = micros();
    tsf_render_short(g_tsf, buf + offset, (int)framesSeg, 0);
    stats.synthUs += micros() - synthStartUs;
    ++stats.synthSegments;
    double actualMs = framesSeg * 1000.0 / g_sampleRate;
    g_midiMs += actualMs;
    g_midiTick += actualMs * (1000.0 * g_midiDivision) / g_midiTempoUs;
    offset += framesSeg;
  }
  xSemaphoreGive(g_tsfMutex);

  midiDecayVisuals();
  if (!midiHasPendingEvents()) {
    g_playing = false;
    g_midiLoaded = false;
    queueNextTrackAdvance();
  }
  return stats;
}

// ══════════════════════════════════════════════════════════════════════════════
// AUDIO INIT
// ══════════════════════════════════════════════════════════════════════════════

static bool initAudio() {
  g_audioReady = false;
  g_i2sPort = I2S_NUM_0;

  switch (g_audioMode) {

    case AUDIO_ES8311: {
      // Align with the known-working MP3 player path on Cardputer ADV:
      // explicitly start Speaker and drive ES8311 through M5Unified.
      g_sampleRate = 22050;
      M5Cardputer.Speaker.begin();
      M5Cardputer.Speaker.setVolume(255);
      M5Cardputer.Speaker.setAllChannelVolume(255);
      resetEs8311Buffers(true);
      g_audioReady = true;
      Serial.printf("[AUDIO] ES8311  %uHz mono via M5Cardputer.Speaker\r\n", g_sampleRate);
      return true;
    }

    case AUDIO_I2S_DAC: {
      g_sampleRate = 22050;
      g_i2sPort = I2S_NUM_0;
      i2s_config_t c{};
      c.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
      c.sample_rate          = g_sampleRate;
      c.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
      c.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
      c.communication_format = I2S_COMM_FORMAT_STAND_I2S;
      c.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
      c.dma_buf_count        = I2S_DMA_BUFS;
      c.dma_buf_len          = I2S_DMA_LEN;
      c.tx_desc_auto_clear   = true;
      esp_err_t err = i2s_driver_install(g_i2sPort, &c, 0, nullptr);
      if (err != ESP_OK) {
        Serial.printf("[AUDIO] I2S DAC driver install failed: %d\r\n", (int)err);
        return false;
      }
      i2s_pin_config_t p{};
      p.bck_io_num    = PIN_I2S_BCK;
      p.ws_io_num     = PIN_I2S_WS;
      p.data_out_num  = PIN_I2S_DOUT;
      p.data_in_num   = I2S_PIN_NO_CHANGE;
      err = i2s_set_pin(g_i2sPort, &p);
      if (err != ESP_OK) {
        Serial.printf("[AUDIO] I2S DAC pin setup failed: %d\r\n", (int)err);
        i2s_driver_uninstall(g_i2sPort);
        return false;
      }
      i2s_zero_dma_buffer(g_i2sPort);
      g_audioReady = true;
      Serial.printf("[AUDIO] I2S DAC  %uHz stereo on I2S%d\r\n", g_sampleRate, (int)g_i2sPort);
      return true;
    }

    case AUDIO_PDM: {
      g_sampleRate = 16000;
      g_i2sPort = I2S_NUM_0;
      i2s_config_t c{};
      c.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_PDM);
      c.sample_rate          = g_sampleRate;
      c.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
      c.channel_format       = I2S_CHANNEL_FMT_ONLY_RIGHT;
      c.communication_format = I2S_COMM_FORMAT_STAND_PCM_SHORT;
      c.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1;
      c.dma_buf_count        = I2S_DMA_BUFS;
      c.dma_buf_len          = I2S_DMA_LEN;
      c.tx_desc_auto_clear   = true;
      esp_err_t err = i2s_driver_install(g_i2sPort, &c, 0, nullptr);
      if (err != ESP_OK) {
        Serial.printf("[AUDIO] PDM driver install failed: %d\r\n", (int)err);
        return false;
      }
      i2s_pin_config_t p{};
      p.bck_io_num    = PIN_PDM_CLK;
      p.ws_io_num     = I2S_PIN_NO_CHANGE;
      p.data_out_num  = PIN_SPEAKER;
      p.data_in_num   = I2S_PIN_NO_CHANGE;
      err = i2s_set_pin(g_i2sPort, &p);
      if (err != ESP_OK) {
        Serial.printf("[AUDIO] PDM pin setup failed: %d\r\n", (int)err);
        i2s_driver_uninstall(g_i2sPort);
        return false;
      }
      i2s_zero_dma_buffer(g_i2sPort);
      g_audioReady = true;
      Serial.printf("[AUDIO] PDM GPIO%d  %uHz mono on I2S%d\r\n", PIN_SPEAKER, g_sampleRate, (int)g_i2sPort);
      return true;
    }

    case AUDIO_PWM: {
      g_sampleRate = 16000;
      ledcSetup(LEDC_CH, LEDC_FREQ, LEDC_BITS);
      ledcAttachPin(PIN_SPEAKER, LEDC_CH);
      ledcWrite(LEDC_CH, 128);
      g_pwmTimer = timerBegin(0, 80, true);
      timerAttachInterrupt(g_pwmTimer, &pwmISR, true);
      timerAlarmWrite(g_pwmTimer, 1000000UL / g_sampleRate, true);
      timerAlarmEnable(g_pwmTimer);
      g_audioReady = true;
      Serial.printf("[AUDIO] PWM GPIO%d  %uHz mono\r\n", PIN_SPEAKER, g_sampleRate);
      return true;
    }

    default: break;
  }

  return false;
}

// ══════════════════════════════════════════════════════════════════════════════
// AUDIO TASK  (Core 0, max priority)
// ══════════════════════════════════════════════════════════════════════════════

static void monoToStereoDup(const int16_t* mono, int16_t* stereo, int n) {
  for (int i = 0; i < n; i++) {
    stereo[i * 2]     = mono[i];
    stereo[i * 2 + 1] = mono[i];
  }
}

static void audioTask(void*) {
  size_t written;
  while (true) {
    // ES8311 audio is handled in the main loop (Core 1) to avoid
    // cross-core access to M5Unified Speaker which corrupts I2C/audio.
    if (!g_audioReady || g_audioMode == AUDIO_ES8311) {
      vTaskDelay(20 / portTICK_PERIOD_MS);
      continue;
    }

    bool shouldRender = g_tsf && g_playing;

    if (shouldRender) {
      xSemaphoreTake(g_tsfMutex, portMAX_DELAY);
      if (useStereoSynthOutput()) {
        tsf_render_short(g_tsf, g_i2sBuf, CHUNK_FRAMES, 0);
      } else {
        tsf_render_short(g_tsf, g_audioBuf, CHUNK_FRAMES, 0);
      }
      xSemaphoreGive(g_tsfMutex);
    } else {
      memset(g_audioBuf, 0, CHUNK_FRAMES * sizeof(int16_t));
      memset(g_i2sBuf, 0, sizeof(g_i2sBuf));
    }

    switch (g_audioMode) {
      case AUDIO_ES8311:
        // Handled in main loop, should not reach here.
        vTaskDelay(20 / portTICK_PERIOD_MS);
        break;
      case AUDIO_I2S_DAC:
        i2s_write(g_i2sPort, g_i2sBuf, CHUNK_FRAMES * 4, &written, portMAX_DELAY);
        break;
      case AUDIO_PDM:
        i2s_write(g_i2sPort, g_audioBuf, CHUNK_FRAMES * 2, &written, portMAX_DELAY);
        break;
      case AUDIO_PWM:
        g_pwmDone = false; g_pwmIdx = 0;
        while (!g_pwmDone) taskYIELD();
        break;
      default:
        vTaskDelay(10 / portTICK_PERIOD_MS);
        break;
    }
  }
}

static void playerUiStep() {
  static cardputer_keyboard::KeysState prevKeys{};
  static uint32_t uiCallN = 0;

  static uint32_t updDbgN = 0;
  ++updDbgN;
  M5Cardputer.update();
  bool isChg = M5Cardputer.Keyboard.isChange();
  auto ks    = M5Cardputer.Keyboard.keysState();
  uint32_t now = millis();

  if (g_idleSplashActive) {
    if (isChg || keysStateHasAnyInput(ks)) {
      g_idleSplashActive = false;
      g_lastPlaybackActivityMs = now;
      g_lastUiMs = now;
      invalidateUiCache();
      g_fullRedraw = true;
      g_needRedraw = true;
    }
    prevKeys = ks;
    return;
  }

  if (g_playing) {
    g_lastPlaybackActivityMs = now;
  } else if (!g_trackAdvancePending
          && g_lastPlaybackActivityMs != 0
          && (uint32_t)(now - g_lastPlaybackActivityMs) >= IDLE_SPLASH_DELAY_MS) {
    showBootSplashImage(0);
    g_idleSplashActive = true;
    g_lastUiMs = now;
    prevKeys = ks;
    return;
  }

  uint32_t nowUs = micros();
  uint32_t deltaUs = nowUs - g_lastTickUs;
  g_lastTickUs = nowUs;
  if (currentHeaderTitleScrollIndex() != g_lastHeaderTitleScroll) {
    g_needRedraw = true;
  }

  if (g_audioMode != AUDIO_ES8311 && deltaUs > 0 && deltaUs < 200000) {
    tickMidi(deltaUs / 1000.0);
  }

  if (g_trackAdvancePending && (int32_t)(millis() - g_trackAdvanceAtMs) >= 0) {
    clearTrackAdvance();
    int nextIdx = g_looping ? g_midiIdx
                            : (g_midiIdx + 1) % (int)g_midiList.size();
    saveCurrentMidiConfig(nextIdx);
    loadMidi(nextIdx);
    return;
  }

  // Keyboard detection.
  // isChange() may silently fail on ADV when ES8311 audio is active: the I2S
  // peripheral or the speaker task can affect the TCA8418 INT GPIO, preventing
  // the interrupt from firing.  Manual comparison with prevKeys is a reliable
  // fallback: keysState() still reflects the real hardware state via I2C polling.
  bool kbChanged = isChg
                || (ks.space != prevKeys.space)
                || (ks.enter != prevKeys.enter)
                || (ks.fn    != prevKeys.fn)
                || (ks.del   != prevKeys.del)
                || (ks.hid_keys != prevKeys.hid_keys)
                || (ks.word  != prevKeys.word);

  if (kbChanged) {
    if (cardputer_keyboard::pressed_space(ks, prevKeys)) {
      setPlaybackState(!g_playing);
    }

    if (cardputer_keyboard::pressed_nav_right(ks, prevKeys)) {
      g_midiIdx = (g_midiIdx + 1) % (int)g_midiList.size();
      saveCurrentMidiConfig(g_midiIdx);
      loadMidi(g_midiIdx);
    } else if (cardputer_keyboard::pressed_nav_left(ks, prevKeys)) {
      g_midiIdx = (g_midiIdx - 1 + (int)g_midiList.size()) % (int)g_midiList.size();
      saveCurrentMidiConfig(g_midiIdx);
      loadMidi(g_midiIdx);
    } else if (cardputer_keyboard::pressed_nav_up(ks, prevKeys)) {
      g_vol_dB = (g_vol_dB + 3.0f < 0.0f) ? g_vol_dB + 3.0f : 0.0f;
      if (g_tsf) {
        xSemaphoreTake(g_tsfMutex, portMAX_DELAY);
        tsf_set_output(g_tsf, currentSynthOutputMode(), (int)g_sampleRate, g_vol_dB);
        xSemaphoreGive(g_tsfMutex);
      }
      Serial.printf("[VOL] %.0f dB\r\n", g_vol_dB);
    } else if (cardputer_keyboard::pressed_nav_down(ks, prevKeys)) {
      g_vol_dB = (g_vol_dB - 3.0f > -40.0f) ? g_vol_dB - 3.0f : -40.0f;
      if (g_tsf) {
        xSemaphoreTake(g_tsfMutex, portMAX_DELAY);
        tsf_set_output(g_tsf, currentSynthOutputMode(), (int)g_sampleRate, g_vol_dB);
        xSemaphoreGive(g_tsfMutex);
      }
      Serial.printf("[VOL] %.0f dB\r\n", g_vol_dB);
    }

    for (char c : ks.word) {
      if (c == 'r' || c == 'R') { loadMidi(g_midiIdx); break; }
      if (c == 'l' || c == 'L') { g_looping = !g_looping; g_needRedraw = true; }
      if (c == '+' || c == '=') {
        g_vol_dB = (g_vol_dB + 3.0f < 0.0f) ? g_vol_dB + 3.0f : 0.0f;
        if (g_tsf) {
          xSemaphoreTake(g_tsfMutex, portMAX_DELAY);
          tsf_set_output(g_tsf, currentSynthOutputMode(), (int)g_sampleRate, g_vol_dB);
          xSemaphoreGive(g_tsfMutex);
        }
        Serial.printf("[VOL] %.0f dB\r\n", g_vol_dB);
      }
      if (c == '-' || c == '_') {
        g_vol_dB = (g_vol_dB - 3.0f > -40.0f) ? g_vol_dB - 3.0f : -40.0f;
        if (g_tsf) {
          xSemaphoreTake(g_tsfMutex, portMAX_DELAY);
          tsf_set_output(g_tsf, currentSynthOutputMode(), (int)g_sampleRate, g_vol_dB);
          xSemaphoreGive(g_tsfMutex);
        }
        Serial.printf("[VOL] %.0f dB\r\n", g_vol_dB);
      }
      if (c == 'm' || c == 'M') { openFileSelector(); break; }
    }

    prevKeys = ks;
  }

  if (now - g_lastUiMs >= 66) {
    g_lastUiMs = now;
    if (g_fullRedraw) {
      redrawFull();
    } else if (g_needRedraw) {
      redrawPartial();
    }
  }
  if (uiCallN < 10) { uiCallN++; }
}

// ══════════════════════════════════════════════════════════════════════════════
// SF2 LOAD
// ══════════════════════════════════════════════════════════════════════════════

static bool sf2HasValidHeader(File& file) {
  uint8_t hdr[12];
  if (!file.seek(0)) return false;
  int n = (int)file.read(hdr, sizeof(hdr));
  file.seek(0);
  return n == (int)sizeof(hdr)
      && hdr[0] == 'R' && hdr[1] == 'I' && hdr[2] == 'F' && hdr[3] == 'F'
      && hdr[8] == 's' && hdr[9] == 'f' && hdr[10] == 'b' && hdr[11] == 'k';
}

static void logSF2LoadFailure(const char* path,
                              size_t fileSize,
                              bool headerOk,
                              uint32_t freeBefore,
                              uint32_t largestBefore,
                              uint32_t freeAfter,
                              uint32_t largestAfter) {
  Serial.printf("[SF2] FAIL path=%s size=%u free_before=%u largest_before=%u free_after=%u largest_after=%u\r\n",
                path, (unsigned)fileSize, freeBefore, largestBefore, freeAfter, largestAfter);

  if (!headerOk) {
    Serial.println("[SF2] Reason: the file header is not RIFF/sfbk, so it does not look like a valid SF2.");
    return;
  }

  if (fileSize > freeBefore) {
    Serial.printf("[SF2] Reason: the file size (%u bytes) is larger than the total free heap before load (%u bytes).\r\n",
                  (unsigned)fileSize, freeBefore);
  }

  if (fileSize > largestBefore) {
    Serial.printf("[SF2] Reason: the file size (%u bytes) is larger than the biggest contiguous free heap block (%u bytes).\r\n",
                  (unsigned)fileSize, largestBefore);
    Serial.println("[SF2] Fragmentation or non-contiguous chunks are likely involved.");
  }

  if (fileSize <= freeBefore && fileSize <= largestBefore) {
    Serial.println("[SF2] Reason: tsf_load failed even though the raw file size appears to fit.");
    Serial.println("[SF2] The SF2 may be invalid/corrupted, or TinySoundFont may need extra allocations and larger contiguous chunks than the file size alone suggests.");
  }
}

static bool loadSF2(const char* path) {
  if (g_tsf) { tsf_close(g_tsf); g_tsf = nullptr; }
  uint32_t freeBefore    = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
  uint32_t largestBefore = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
  uint32_t minBefore     = heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
  Serial.printf("[SF2] Loading %s\r\n", path);
  Serial.printf("[SF2] Heap before: free=%u largest=%u min=%u\r\n",
                freeBefore, largestBefore, minBefore);

  g_sf2File = SD.open(path, FILE_READ);
  if (!g_sf2File) {
    Serial.println("[SF2] FAIL: file open failed");
    return false;
  }

  size_t fileSize = (size_t)g_sf2File.size();
  bool headerOk = sf2HasValidHeader(g_sf2File);
  Serial.printf("[SF2] File size=%u bytes\r\n", (unsigned)fileSize);
  if (!headerOk) {
    g_sf2File.close();
    logSF2LoadFailure(path, fileSize, false,
                      freeBefore, largestBefore,
                      heap_caps_get_free_size(MALLOC_CAP_DEFAULT),
                      heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
    return false;
  }

  tsf_stream s{ nullptr, sf2_read, sf2_skip };
  g_tsf = tsf_load(&s);
  g_sf2File.close();
  uint32_t freeAfter    = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
  uint32_t largestAfter = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
  if (!g_tsf) {
    logSF2LoadFailure(path, fileSize, true,
                      freeBefore, largestBefore,
                      freeAfter, largestAfter);
    return false;
  }

  tsf_set_output(g_tsf, currentSynthOutputMode(), (int)g_sampleRate, g_vol_dB);
  tsf_set_max_voices(g_tsf, MAX_VOICES);
  Serial.printf("[SF2] OK preset=%d size=%u free_before=%u largest_before=%u free_after=%u largest_after=%u\r\n",
                tsf_get_presetcount(g_tsf), (unsigned)fileSize,
                freeBefore, largestBefore, freeAfter, largestAfter);
  return true;
}

// ══════════════════════════════════════════════════════════════════════════════
// MIDI
// ══════════════════════════════════════════════════════════════════════════════

static void resetChannels() {
  for (int i = 0; i < 16; i++) {
    g_ch[i] = ChannelInfo{};
    g_ch[i].volume = 100;
    g_ch[i].isDrum = (i == 9);
  }
  markAllChannelsDirty();
}

static bool midiTryBufferInRam(size_t midiFileSize, uint32_t largestBefore) {
  midiReleaseRamBuffer();

  if (midiFileSize == 0) return false;
  uint32_t needWithHeadroom = (uint32_t)midiFileSize + MIDI_RAM_HEADROOM;
  if (needWithHeadroom > largestBefore) {
    Serial.printf("[MIDI] RAM copy skipped: need %u bytes + %u headroom, largest block is %u\r\n",
                  (unsigned)midiFileSize,
                  (unsigned)MIDI_RAM_HEADROOM,
                  (unsigned)largestBefore);
    return false;
  }

  uint8_t* buf = (uint8_t*)heap_caps_malloc(midiFileSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!buf) {
    Serial.printf("[MIDI] RAM copy skipped: internal allocation of %u bytes failed\r\n",
                  (unsigned)midiFileSize);
    return false;
  }

  if (!g_midiFile.seek(0)) {
    heap_caps_free(buf);
    Serial.println("[MIDI] RAM copy skipped: could not rewind file");
    return false;
  }

  size_t n = g_midiFile.read(buf, midiFileSize);
  if (n != midiFileSize) {
    heap_caps_free(buf);
    Serial.printf("[MIDI] RAM copy skipped: short read (%u/%u)\r\n",
                  (unsigned)n, (unsigned)midiFileSize);
    g_midiFile.seek(0);
    return false;
  }

  g_midiData = buf;
  g_midiDataSize = midiFileSize;
  g_midiFromRam = true;
  g_midiFile.close();
  Serial.printf("[MIDI] Buffered in RAM: size=%u free=%u largest=%u\r\n",
                (unsigned)midiFileSize,
                (unsigned)heap_caps_get_free_size(MALLOC_CAP_DEFAULT),
                (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
  return true;
}

static void setPlaybackState(bool playing) {
  g_playing = playing;
  g_lastPlaybackActivityMs = millis();
  if (playing) {
    g_idleSplashActive = false;
  }
  if (!playing && g_tsf) {
    xSemaphoreTake(g_tsfMutex, portMAX_DELAY);
    tsf_note_off_all(g_tsf);
    xSemaphoreGive(g_tsfMutex);
  }
  if (!playing) {
    resetEs8311Buffers(true);
  }
  g_fullRedraw = true;
  g_needRedraw = true;
}

static void saveCurrentMidiConfig(int idx) {
  if (idx < 0 || idx >= (int)g_midiList.size()) return;
  g_cfg.midiIdx = idx;
  g_cfg.midiPath = g_midiList[idx];
  saveConfig(g_cfg);
}

static void shutdownAudio() {
  bool wasReady = g_audioReady;
  g_audioReady = false;
  if (wasReady) {
    delay(20);
  }

  switch (g_audioMode) {
    case AUDIO_ES8311:
      resetEs8311Buffers(true);
      M5Cardputer.Speaker.stop(0);
      break;

    case AUDIO_I2S_DAC:
    case AUDIO_PDM:
      i2s_zero_dma_buffer(g_i2sPort);
      i2s_driver_uninstall(g_i2sPort);
      break;

    case AUDIO_PWM:
      if (g_pwmTimer) {
        timerAlarmDisable(g_pwmTimer);
        timerEnd(g_pwmTimer);
        g_pwmTimer = nullptr;
      }
      ledcWrite(LEDC_CH, 128);
      break;

    default:
      break;
  }
}

// Scan the folder into g_midiList and set g_midiIdx to the target file.
static void buildMidiList(const String& folder, const String& target) {
  // Scans the folder into g_midiList and sets g_midiIdx to the target file.
  g_midiList.clear();
  File dir = SD.open(folder.c_str());
  if (dir) {
    File f;
    while ((f = dir.openNextFile())) {
      String nm = String(f.name());
      String nl = nm; nl.toLowerCase();
      if (nl.endsWith(".mid") || nl.endsWith(".midi"))
        g_midiList.push_back(folder + "/" + nm);
      f.close();
    }
    dir.close();
  }
  if (g_midiList.empty()) g_midiList.push_back(target);
  g_midiIdx = 0;
  for (int i = 0; i < (int)g_midiList.size(); i++)
    if (g_midiList[i] == target) { g_midiIdx = i; break; }
}

static bool loadMidi(int idx) {
  if (idx < 0 || idx >= (int)g_midiList.size()) return false;

  g_playing = false;
  resetEs8311Buffers(true);
  if (g_tsf) {
    xSemaphoreTake(g_tsfMutex, portMAX_DELAY);
    tsf_note_off_all(g_tsf);
    xSemaphoreGive(g_tsfMutex);
  }
  delay(60);

  midiCloseStream();
  resetChannels();

  auto fail = [&]() -> bool {
    midiCloseStream();
    return false;
  };

  g_midiFile = SD.open(g_midiList[idx].c_str());
  if (!g_midiFile) { Serial.println("[MIDI] File not found"); return false; }
  size_t midiFileSize = (size_t)g_midiFile.size();
  uint32_t freeBefore = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
  uint32_t largestBefore = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
  Serial.printf("[MIDI] Parsing %s (%u bytes) heap_free=%u largest=%u\r\n",
                g_midiList[idx].c_str(), (unsigned)midiFileSize,
                (unsigned)freeBefore,
                (unsigned)largestBefore);

  if (!midiTryBufferInRam(midiFileSize, largestBefore)) {
    g_midiFromRam = false;
    if (!g_midiFile.seek(0)) {
      Serial.println("[MIDI] Failed to rewind file for SD stream");
      return fail();
    }
    Serial.println("[MIDI] Using SD stream");
  }

  std::vector<MidiTrackDescriptor> descs;
  uint16_t format = 0;
  uint16_t division = 0;
  uint32_t firstEventMs = 0;
  uint8_t* midiBuffer = nullptr;
  if (!midiReadDescriptors(descs, format, division)) {
    return fail();
  }

  if (midiScanDuration(descs, division, g_songDurMs, firstEventMs)) {
    if (g_songDurMs > 0) {
      Serial.printf("[MIDI] Duration: %us first_evt=%ums\r\n",
                    (unsigned)(g_songDurMs / 1000),
                    (unsigned)firstEventMs);
    }
  } else {
    g_songDurMs = 0;
  }

  midiPrimeTracks(descs, g_midiTracks);
  if (!midiHasPendingEvents()) {
    Serial.println("[MIDI] No playable events found");
    return fail();
  }

  g_midiLoaded = true;
  g_midiDivision = division;
  g_midiTempoUs = 500000;
  g_midiTick = 0.0;
  g_midiMs = 0.0;
  g_midiIdx = idx;
  g_playing = true;
  g_lastPlaybackActivityMs = millis();
  g_idleSplashActive = false;
  clearTrackAdvance();
  resetEs8311Buffers(false);
  invalidateUiCache();
  g_fullRedraw = true;
  Serial.printf("[MIDI] %s  stream=1 src=%s tracks=%u fmt=%u dur=%us\r\n",
                g_midiList[idx].c_str(),
                g_midiFromRam ? "RAM" : "SD",
                (unsigned)g_midiTracks.size(),
                (unsigned)format,
                (unsigned)(g_songDurMs / 1000));
  return true;

  std::vector<MidiTrackDescriptor> legacyDescs;
  uint16_t legacyFormat = 0;
  uint16_t legacyDivision = 0;
  if (!midiReadDescriptors(legacyDescs, legacyFormat, legacyDivision)) {
    return fail();
  }

  // TML builds a linked-list of tml_message structs — typically 6–8× the raw
  // file size. If the heap is too small, tml_load_memory() writes past the end
  // of its realloc'd buffer and corrupts the heap (CORRUPT HEAP panic).
  // Guard: require at least 7× the file size as the largest free block.
  {
    uint32_t tmlNeed = (uint32_t)midiFileSize * 7;
    if (largestBefore < tmlNeed) {
      Serial.printf("[MIDI] Skipping: need ~%u bytes (7× file) for TML event list, "
                    "only %u largest-free available\r\n",
                    tmlNeed, largestBefore);
      heap_caps_free(midiBuffer);
      return false;
    }
  }

  g_midiRoot = tml_load_memory(midiBuffer, (int)midiFileSize);
  heap_caps_free(midiBuffer);
  if (!g_midiRoot) {
    Serial.printf("[MIDI] tml_load FAILED heap_free=%u largest=%u\r\n",
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_DEFAULT),
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT));
    return false;
  }

  tml_message* last = g_midiRoot;
  while (last->next) last = last->next;
  g_songDurMs = (uint32_t)last->time;

  g_midiCur   = g_midiRoot;
  g_midiMs    = 0.0;
  g_midiIdx   = idx;
  g_playing   = true;
  g_fullRedraw = true;
  Serial.printf("[MIDI] %s  dur=%us first_evt=%ums\r\n",
                g_midiList[idx].c_str(),
                g_songDurMs / 1000,
                (unsigned)g_midiRoot->time);
  return true;
}

// ══════════════════════════════════════════════════════════════════════════════
// MIDI TICK
// ══════════════════════════════════════════════════════════════════════════════

static void tickMidi(double deltaMs) {
  if (!g_playing || !g_midiLoaded || !g_tsf || g_midiDivision == 0) return;
  double remainingMs = deltaMs;

  while (remainingMs > 0.0 && midiHasPendingEvents()) {
    int best = midiFindNextTrack(g_midiTracks);
    if (best < 0) break;

    uint32_t nextTick = g_midiTracks[best].event.tick;
    if ((double)nextTick > g_midiTick) {
      double msToNext = ((double)nextTick - g_midiTick) * g_midiTempoUs
                      / (1000.0 * g_midiDivision);
      if (msToNext > remainingMs) {
        g_midiMs += remainingMs;
        g_midiTick += remainingMs * (1000.0 * g_midiDivision) / g_midiTempoUs;
        remainingMs = 0.0;
        break;
      }
      g_midiMs += msToNext;
      remainingMs -= msToNext;
      g_midiTick = (double)nextTick;
    } else {
      g_midiTick = (double)nextTick;
    }

    xSemaphoreTake(g_tsfMutex, portMAX_DELAY);
    for (auto& tr : g_midiTracks) {
      if (!tr.hasEvent || tr.event.tick != nextTick) continue;
      midiProcessEvent(tr.event);
      midiTrackReadNextEvent(tr);
    }
    xSemaphoreGive(g_tsfMutex);
  }

  if (remainingMs > 0.0) g_midiMs += remainingMs;

  midiDecayVisuals();

  if (!midiHasPendingEvents()) {
    g_playing = false;
    g_midiLoaded = false;
    queueNextTrackAdvance();
  }
  return;

  if (!g_playing || !g_midiRoot || !g_tsf) return;
  g_midiMs += deltaMs;

  xSemaphoreTake(g_tsfMutex, portMAX_DELAY);
  for (; g_midiCur; g_midiCur = g_midiCur->next) {
    if (g_midiMs < g_midiCur->time) break;
    int ch = g_midiCur->channel;
    if (ch < 0 || ch > 15) continue;

    switch (g_midiCur->type) {
      case TML_PROGRAM_CHANGE:
        g_ch[ch].program = (uint8_t)g_midiCur->program;
        tsf_channel_set_presetnumber(g_tsf, ch, g_midiCur->program, ch == 9);
        markChannelDirty(ch);
        break;

      case TML_NOTE_ON:
        if (g_midiCur->velocity > 0) {
          g_ch[ch].note      = (uint8_t)g_midiCur->key;
          g_ch[ch].velocity  = (uint8_t)g_midiCur->velocity;
          g_ch[ch].active    = true;
          g_ch[ch].lastNoteMs = millis();
          if (g_ch[ch].noteCount < 255) g_ch[ch].noteCount++;
          g_ch[ch].noteHistory[g_ch[ch].histIdx & 3] = (uint8_t)g_midiCur->key;
          g_ch[ch].histIdx++;
          tsf_channel_note_on(g_tsf, ch, g_midiCur->key,
                              g_midiCur->velocity / 127.0f);
        } else {
          if (g_ch[ch].noteCount > 0) g_ch[ch].noteCount--;
          if (!g_ch[ch].noteCount) g_ch[ch].active = false;
          tsf_channel_note_off(g_tsf, ch, g_midiCur->key);
        }
        markChannelDirty(ch);
        break;

      case TML_NOTE_OFF:
        if (g_ch[ch].noteCount > 0) g_ch[ch].noteCount--;
        if (!g_ch[ch].noteCount) g_ch[ch].active = false;
        tsf_channel_note_off(g_tsf, ch, g_midiCur->key);
        g_needRedraw = true;
        break;

      case TML_PITCH_BEND:
        tsf_channel_set_pitchwheel(g_tsf, ch, g_midiCur->pitch_bend);
        break;

      case TML_CONTROL_CHANGE:
        tsf_channel_midi_control(g_tsf, ch,
                                 g_midiCur->control,
                                 g_midiCur->control_value);
        if (g_midiCur->control == 7)
          g_ch[ch].volume = (uint8_t)g_midiCur->control_value;
        break;
    }
  }
  xSemaphoreGive(g_tsfMutex);

  // Visual decay for channel activity.
  uint32_t now = millis();
  for (int i = 0; i < 16; i++) {
    if (g_ch[i].active && (now - g_ch[i].lastNoteMs) > 600) {
      g_ch[i].active    = false;
      g_ch[i].noteCount = 0;
      markChannelDirty(i);
    }
  }

  // End of track -> advance automatically.
  if (!g_midiCur) {
    g_playing = false;
    delay(600);
    int next = g_looping ? g_midiIdx
                         : (g_midiIdx + 1) % (int)g_midiList.size();
    g_cfg.midiIdx = next;
    saveConfig(g_cfg);
    loadMidi(next);
  }
}

// ══════════════════════════════════════════════════════════════════════════════
// DISPLAY
// ══════════════════════════════════════════════════════════════════════════════

// Main display palette
#define C_BG      ((uint16_t)0x0000)
#define C_HDR_BG  ((uint16_t)0x0318)
#define C_ACTIVE  ((uint16_t)0x07E0)
#define C_DRUM    ((uint16_t)0xFD00)
#define C_INACT   ((uint16_t)0x18C3)
#define C_TEXT    ((uint16_t)0xFFFF)
#define C_DIM     ((uint16_t)0x7BEF)
#define C_DARKER  ((uint16_t)0x2945)
#define C_CYAN    ((uint16_t)0x07FF)
#define C_BAR_BG  ((uint16_t)0x1082)
#define C_PROG    ((uint16_t)0x001F)
#define C_RED     ((uint16_t)0xF800)
#define C_ACCENT  ((uint16_t)0x07FF)
#define C_WARN    ((uint16_t)0xFBE0)
#define C_SEP     ((uint16_t)0x4208)

static const int ROW_H  = 14;
static const int ROW_Y0 = 13;
static const int COL_W  = 120;

static void drawHeader() {
  auto& d = M5Cardputer.Display;
  d.fillRect(0, 0, 240, 12, C_HDR_BG);
  d.setTextColor(C_TEXT, C_HDR_BG);
  d.setTextSize(1);

  d.setCursor(2, 2);
  d.print(g_playing ? "\x10 " : "|| ");
  d.print(g_looping ? "[L]" : "   ");

  char info[26];
  buildHeaderInfoText(info, sizeof(info));
  d.setCursor(240 - (int)strlen(info)*6 - 2, 2);
  d.print(info);

  int titleX = 38;
  int infoX = 240 - (int)strlen(info) * 6 - 2;
  int titleW = infoX - titleX - 4;
  if (titleW > 0) {
    d.fillRect(titleX, 0, titleW, 12, C_HDR_BG);
    d.setCursor(titleX, 2);
    d.print(currentHeaderTitleWindow());
  }
}

static void drawProgressBar() {
  auto& d = M5Cardputer.Display;
  const int y = 125;
  d.fillRect(0, y, 240, 10, C_BAR_BG);
  if (g_songDurMs > 0 && g_midiMs > 0) {
    int w = (int)(240.0 * g_midiMs / (double)g_songDurMs);
    if (w > 240) w = 240;
    d.fillRect(0, y, w, 3, C_PROG);
  }
  d.setTextColor(C_DIM, C_BAR_BG);
  d.setCursor(1, y + 3);
  d.print("SPC:play  ,/:trk  ;.:vol  L:loop  M:Menu");
}

static void drawChannelRow(int ch) {
  auto& d = M5Cardputer.Display;
  int col = ch / 8, row = ch % 8;
  int x   = col * COL_W;
  int y   = ROW_Y0 + row * ROW_H;

  const ChannelInfo& ci = g_ch[ch];
  bool active = ci.active;
  bool drum   = ci.isDrum || (ch == 9);

  // Row background
  uint16_t rowBg = (row & 1) ? C_DARKER : C_BG;
  if (active) rowBg = drum ? (uint16_t)0x2800 : (uint16_t)0x0030;
  d.fillRect(x, y, COL_W, ROW_H, rowBg);

  // Channel badge
  uint16_t bc = active ? (drum ? C_DRUM : C_ACTIVE) : C_INACT;
  d.fillRect(x, y, 13, ROW_H, bc);
  d.setTextColor(C_TEXT, bc);
  d.setTextSize(1);
  d.setCursor(x + 1, y + 3);
  d.printf("%2d", ch + 1);

  // Nome strumento
  d.setTextColor(active ? C_TEXT : C_DIM, rowBg);
  d.setCursor(x + 15, y + 3);
  {
    char nb[9] = {};
    const char* nm = drum ? "DrumKit"
                          : (ci.program < 128 ? GM_NAMES[ci.program] : "---");
    strncpy(nb, nm, 8);
    d.print(nb);
  }

  // Mini piano-roll (35 px)
  int rx = x + 15 + 52;
  d.fillRect(rx, y, 35, ROW_H, C_BAR_BG);
  if (active) {
    uint16_t nc = drum ? C_DRUM : C_CYAN;
    for (int k = 0; k < 4; k++) {
      uint8_t n = ci.noteHistory[k];
      if (n > 0) {
        int     nx  = rx + n * 35 / 128;
        uint8_t age = (ci.histIdx - k) & 3;
        uint16_t c2 = (age == 0) ? nc : (uint16_t)((nc >> 1) & 0x7BEF);
        d.fillRect(nx, y + 2, 2, ROW_H - 4, c2);
      }
    }
  }

  // Barra velocity (18 px)
  int vx = rx + 35;
  d.fillRect(vx, y + 2, 18, ROW_H - 4, C_BAR_BG);
  if (active && ci.velocity > 0) {
    int      vw = ci.velocity * 18 / 127;
    uint16_t vc = (ci.velocity > 100) ? (uint16_t)0xFB40 : C_CYAN;
    d.fillRect(vx, y + 2, vw, ROW_H - 4, vc);
  }

  // Vertical separator between the two columns
  if (col == 0) d.drawFastVLine(COL_W - 1, y, ROW_H, C_SEP);
}

static void redrawFull() {
  auto& d = M5Cardputer.Display;
  d.startWrite();
  d.fillScreen(C_BG);
  drawHeader();
  for (int i = 0; i < 16; i++) drawChannelRow(i);
  drawProgressBar();
  d.endWrite();
  syncUiCache();
  g_dirtyChannelMask = 0;
  g_fullRedraw = false;
  g_needRedraw = false;
}

static void redrawPartial() {
  bool headerDirty = g_lastHeaderElapsedSec != (uint32_t)(g_midiMs / 1000.0)
                  || g_lastHeaderTotalSec   != (g_songDurMs / 1000)
                  || g_lastHeaderMidiIdx    != g_midiIdx
                  || g_lastHeaderPlaying    != g_playing
                  || g_lastHeaderLooping    != g_looping
                  || g_lastHeaderTitleScroll != currentHeaderTitleScrollIndex();
  int progressW = currentProgressWidth();
  bool progressDirty = g_lastProgressW != progressW;

  auto& d = M5Cardputer.Display;
  d.startWrite();
  if (headerDirty) drawHeader();
  for (int i = 0; i < 16; i++) {
    if (g_dirtyChannelMask & (uint16_t)(1u << i)) drawChannelRow(i);
  }
  if (progressDirty) drawProgressBar();
  d.endWrite();
  syncUiCache();
  g_dirtyChannelMask = 0;
  g_needRedraw = false;
}

// ══════════════════════════════════════════════════════════════════════════════
// UTILITY DISPLAY
// ══════════════════════════════════════════════════════════════════════════════

static void showError(const char* l1,
                      const char* l2 = nullptr,
                      const char* l3 = nullptr) {
  auto& d = M5Cardputer.Display;
  d.fillScreen(C_RED);
  d.setTextColor(C_TEXT);
  d.setTextSize(2); d.setCursor(10, 18); d.println(l1);
  d.setTextSize(1);
  if (l2) { d.setCursor(10, 50); d.println(l2); }
  if (l3) { d.setCursor(10, 65); d.println(l3); }
}

static void showBusy(const char* msg, const char* sub = nullptr) {
  auto& d = M5Cardputer.Display;
  d.fillScreen((uint16_t)0x000A);
  d.setTextColor(C_DIM, (uint16_t)0x000A);
  d.setTextSize(1);
  d.setCursor(10, 55); d.print(msg);
  if (sub) { d.setCursor(10, 68); d.print(sub); }
}

static void showBootSplashImage(uint32_t holdMs = 2000) {
  auto& d = M5Cardputer.Display;
  size_t jpgLen = (size_t)(kBootSplashJpgEnd - kBootSplashJpgStart);
  if (!jpgLen) return;

  d.fillScreen(C_BG);
  bool drawn = d.drawJpg(kBootSplashJpgStart, (uint32_t)jpgLen, 0, 0, d.width(), d.height());
  Serial.printf("[BOOT] Embedded splash len=%u drawn=%d\r\n", (unsigned)jpgLen, drawn ? 1 : 0);
  if (drawn) {
    delay(holdMs);
  }
}

static String parentDirOrRoot(const String& path) {
  int slash = path.lastIndexOf('/');
  return (slash > 0) ? path.substring(0, slash) : "/";
}

static String tailForUi(const String& path, size_t maxChars = 28) {
  if (path.length() <= (int)maxChars) return path;
  return path.substring(path.length() - (int)maxChars);
}

// ══════════════════════════════════════════════════════════════════════════════
// MENU SELEZIONE MODALITÀ AUDIO
// ══════════════════════════════════════════════════════════════════════════════

static AudioMode selectAudioMode(int saved) {
  // Audio mode selection menu.
  auto& d = M5Cardputer.Display;

  struct Opt {
    AudioMode    mode;
    const char*  key;
    const char*  label;
    const char*  desc;
    const char*  hw;
    uint16_t     col;
  };

#ifndef CARDPUTER_V11
  const Opt opts[] = {
    { AUDIO_ES8311,  "1", "ADV built-in ES8311",
      "22kHz mono - built-in output",
      "3.5mm jack + internal speaker",  0xFFE0 },
    { AUDIO_I2S_DAC, "2", "External I2S DAC",
      "22kHz stereo - external DAC",
      "MAX98357A / PCM5102 on GPIO 6/7/8", 0x07E0 },
    { AUDIO_PDM,     "3", "PDM speaker GPIO2",
      "16kHz mono - no extra hardware",
      "Cardputer built-in speaker",   0x07FF },
    { AUDIO_PWM,     "4", "PWM LEDC GPIO2",
      "16kHz mono - universal fallback",
      "Cardputer built-in speaker",   0xFD20 },
  };
  const int N = 4;
#else
  const Opt opts[] = {
    { AUDIO_I2S_DAC, "1", "External I2S DAC",
      "22kHz stereo - external DAC",
      "MAX98357A / PCM5102 on GPIO 6/7/8", 0x07E0 },
    { AUDIO_PDM,     "2", "PDM speaker GPIO2",
      "16kHz mono - no extra hardware",
      "Cardputer built-in speaker",   0x07FF },
    { AUDIO_PWM,     "3", "PWM LEDC GPIO2",
      "16kHz mono - universal fallback",
      "Cardputer built-in speaker",   0xFD20 },
  };
  const int N = 3;
#endif

  // Find the default index from the saved config.
  int sel = 0;
  for (int i = 0; i < N; i++)
    if ((int)opts[i].mode == saved) { sel = i; break; }

  bool confirmed = false;
  const int rowH = (N == 4) ? 27 : 34;

  auto draw = [&]() {
    d.fillScreen((uint16_t)0x000A);
    d.setTextColor(C_ACCENT, (uint16_t)0x000A); d.setTextSize(1);
    d.setCursor(4, 4); d.print("GM MIDI PLAYER  \xBB  Audio output");
    d.drawFastHLine(0, 14, 240, C_ACCENT);
    for (int i = 0; i < N; i++) {
      int      y = 17 + i * rowH;
      bool     s = (i == sel);
      uint16_t bg = s ? (uint16_t)0x0318 : (uint16_t)0x000A;
      if (s) {
        d.fillRoundRect(2, y, 236, rowH-1, 3, bg);
        d.drawRoundRect(2, y, 236, rowH-1, 3, opts[i].col);
      }
      uint16_t kc = s ? opts[i].col : C_DIM;
      d.fillRoundRect(6, y+4, 14, 12, 2, kc);
      d.setTextColor((uint16_t)0x000A, kc); d.setCursor(9, y+7); d.print(opts[i].key);
      d.setTextColor(s ? C_TEXT : C_DIM, bg);
      d.setCursor(26, y+3); d.print(opts[i].label);
      d.setTextColor(s ? opts[i].col : (uint16_t)0x4208, bg);
      d.setCursor(26, y+12); d.print(opts[i].desc);
      if (rowH >= 34) {
        d.setTextColor(s ? C_WARN : (uint16_t)0x2945, bg);
        d.setCursor(26, y+22); d.print(opts[i].hw);
      }
    }
    d.fillRect(0, 124, 240, 11, (uint16_t)0x1082);
    d.setTextColor(C_DIM, (uint16_t)0x1082);
    d.setCursor(4, 127);
    char footer[40];
    snprintf(footer, sizeof(footer), ";.:nav  1-%d:quick  / or ENT:ok", N);
    d.print(footer);
  };

  draw();
  uint32_t lastBlink = millis(); bool blink = true;
  cardputer_keyboard::KeysState prevKeys{};

  while (!confirmed) {
    M5Cardputer.update();
    if (millis() - lastBlink > 450) {
      lastBlink = millis(); blink = !blink;
      int y = 17 + sel * rowH;
      d.drawRoundRect(2, y, 236, rowH-1, 3,
                      blink ? opts[sel].col : (uint16_t)0x0318);
    }
    if (!M5Cardputer.Keyboard.isChange()) { delay(5); continue; }

    auto ks = M5Cardputer.Keyboard.keysState();
    if (!ks.fn) {
      for (int i = 0; i < N; i++) {
        if (cardputer_keyboard::pressed_digit(ks, prevKeys, static_cast<char>('1' + i))) {
          sel = i;
          confirmed = true;
          break;
        }
      }
    }
    if (!confirmed && (cardputer_keyboard::pressed_word_ci(ks, prevKeys, 'w') ||
                       cardputer_keyboard::pressed_nav_up(ks, prevKeys))) {
      sel = (sel - 1 + N) % N;
      draw();
      lastBlink = millis();
    }
    if (!confirmed && (cardputer_keyboard::pressed_word_ci(ks, prevKeys, 's') ||
                       cardputer_keyboard::pressed_nav_down(ks, prevKeys))) {
      sel = (sel + 1) % N;
      draw();
      lastBlink = millis();
    }
    if (!confirmed && (cardputer_keyboard::pressed_enter(ks, prevKeys) ||
                       cardputer_keyboard::pressed_nav_right(ks, prevKeys))) {
      confirmed = true;
    }
    prevKeys = ks;
    delay(5);
  }

  // Confirmation flash.
  int y = 17 + sel * rowH;
  for (int f = 0; f < 3; f++) {
    d.drawRoundRect(2, y, 236, rowH-1, 3, C_TEXT);  delay(80);
    d.drawRoundRect(2, y, 236, rowH-1, 3, opts[sel].col); delay(80);
  }
  delay(150);
  return opts[sel].mode;
}

static void changeAudioOutputRuntime() {
  AudioMode oldMode = g_audioMode;
  bool wasPlaying = g_playing;
  int currentMidiIdx = g_midiIdx;
  bool hadMidi = !g_midiList.empty() && currentMidiIdx >= 0 && currentMidiIdx < (int)g_midiList.size();

  setPlaybackState(false);
  delay(60);

  AudioMode chosen = selectAudioMode((int)g_audioMode);
  if (chosen == oldMode) {
    if (hadMidi) {
      String midiTail = tailForUi(g_midiList[currentMidiIdx]);
      showBusy("Reloading MIDI...", midiTail.c_str());
      if (loadMidi(currentMidiIdx) && !wasPlaying) {
        setPlaybackState(false);
      }
    } else {
      setPlaybackState(false);
    }
    return;
  }

  showBusy("Switching output...");
  shutdownAudio();
  g_audioMode = chosen;
  g_cfg.audioMode = (int)chosen;
  saveConfig(g_cfg);

  if (!initAudio()) {
    showError("Audio switch failed", "Restoring previous mode");
    delay(1500);
    g_audioMode = oldMode;
    g_cfg.audioMode = (int)oldMode;
    saveConfig(g_cfg);
    if (!initAudio()) {
      showError("Audio restore failed!", "Reboot required");
      delay(2000);
      return;
    }
  }

  if (g_tsf) {
    xSemaphoreTake(g_tsfMutex, portMAX_DELAY);
    tsf_set_output(g_tsf, currentSynthOutputMode(), (int)g_sampleRate, g_vol_dB);
    xSemaphoreGive(g_tsfMutex);
  }

  if (hadMidi) {
    String midiTail = tailForUi(g_midiList[currentMidiIdx]);
    showBusy("Reloading MIDI...", midiTail.c_str());
    if (!loadMidi(currentMidiIdx)) {
      showError("MIDI reload failed", "See serial log");
      delay(1600);
      return;
    }
    if (!wasPlaying) {
      setPlaybackState(false);
    }
  } else {
    setPlaybackState(false);
  }
}

// ══════════════════════════════════════════════════════════════════════════════
// RUNTIME MENU  (key M)
// ══════════════════════════════════════════════════════════════════════════════

static void openFileSelector() {
  // Runtime menu opened with the M key.
  bool wasPlaying = g_playing;
  g_playing = false;
  if (g_tsf) {
    xSemaphoreTake(g_tsfMutex, portMAX_DELAY);
    tsf_note_off_all(g_tsf);
    xSemaphoreGive(g_tsfMutex);
  }
  delay(60);

  auto& d = M5Cardputer.Display;

  // Mini menu: audio / SF2 / MIDI / cancel
  int sel = 0;
  auto drawMenu = [&]() {
    d.fillScreen((uint16_t)0x000A);
    d.setTextColor(C_ACCENT, (uint16_t)0x000A); d.setTextSize(1);
    d.setCursor(4, 4); d.print("Menu:");
    d.drawFastHLine(0, 14, 240, C_ACCENT);
    struct { const char* k; const char* l; uint16_t c; } opts[] = {
      { "1", "Audio Output",       0xFFE0 },
      { "2", "GM Soundfont (.sf2)", 0x07E0 },
      { "3", "MIDI File   (.mid)", 0x07FF },
      { "4", "Cancel",            0x8410 },
    };
    for (int i = 0; i < 4; i++) {
      int y = 18 + i * 28;
      uint16_t bg = (i == sel) ? (uint16_t)0x0318 : (uint16_t)0x1082;
      d.fillRect(4, y, 232, 24, bg);
      d.drawRoundRect(4, y, 232, 24, 3, (i == sel) ? opts[i].c : (uint16_t)0x4208);
      d.fillRoundRect(8, y+7, 16, 13, 2, opts[i].c);
      d.setTextColor((uint16_t)0x000A, opts[i].c); d.setCursor(11, y+10); d.print(opts[i].k);
      d.setTextColor(C_TEXT, bg); d.setCursor(30, y+10); d.print(opts[i].l);
    }
    d.fillRect(0, 123, 240, 12, (uint16_t)0x1082);
    d.setTextColor(C_DIM, (uint16_t)0x1082); d.setCursor(4, 126);
    d.print(";.:nav  / or ENT:ok  , or ESC:back");
  };
  drawMenu();

  int choice = 0;
  cardputer_keyboard::KeysState prevKeys{};
  while (!choice) {
    M5Cardputer.update();
    if (!M5Cardputer.Keyboard.isChange()) { delay(5); continue; }

    auto ks = M5Cardputer.Keyboard.keysState();
    if (!ks.fn) {
      if (cardputer_keyboard::pressed_digit(ks, prevKeys, '1')) choice = 1;
      if (cardputer_keyboard::pressed_digit(ks, prevKeys, '2')) choice = 2;
      if (cardputer_keyboard::pressed_digit(ks, prevKeys, '3')) choice = 3;
      if (cardputer_keyboard::pressed_digit(ks, prevKeys, '4')) choice = 4;
    }
    if (!choice && (cardputer_keyboard::pressed_word_ci(ks, prevKeys, 'w') ||
                    cardputer_keyboard::pressed_nav_up(ks, prevKeys))) {
      sel = (sel + 3) % 4;
      drawMenu();
    }
    if (!choice && (cardputer_keyboard::pressed_word_ci(ks, prevKeys, 's') ||
                    cardputer_keyboard::pressed_nav_down(ks, prevKeys))) {
      sel = (sel + 1) % 4;
      drawMenu();
    }
    if (!choice && (cardputer_keyboard::pressed_enter(ks, prevKeys) ||
                    cardputer_keyboard::pressed_nav_right(ks, prevKeys))) {
      choice = sel + 1;
    }
    if (!choice && (cardputer_keyboard::pressed_escape(ks, prevKeys) ||
                    cardputer_keyboard::pressed_nav_left(ks, prevKeys))) {
      choice = 3;
    }
    prevKeys = ks;
    delay(5);
  }

  if (choice == 1) {
    changeAudioOutputRuntime();
    return;

  } else if (choice == 2) {
    String oldPath = g_cfg.sf2Path;
    String dir = parentDirOrRoot(g_cfg.sf2Path);
    int oldMidiIdx = g_midiIdx;
    bool hadMidi = !g_midiList.empty() && oldMidiIdx >= 0 && oldMidiIdx < (int)g_midiList.size();
    while (true) {
      String chosen = FileSelector::select({ ".sf2", ".SF2" }, g_cfg.sf2Path, "GM Soundfont");
      if (chosen.isEmpty()) break;

      dir = parentDirOrRoot(chosen);
      String tail = tailForUi(chosen);
      midiCloseStream();
      resetChannels();
      showBusy("Loading SF2...", tail.c_str());
      if (loadSF2(chosen.c_str())) {
        g_cfg.sf2Path = chosen;
        saveConfig(g_cfg);
        if (hadMidi) {
          String midiTail = tailForUi(g_midiList[oldMidiIdx]);
          showBusy("Reloading MIDI...", midiTail.c_str());
          if (!loadMidi(oldMidiIdx)) {
            showError("MIDI reload failed", "See serial log");
            delay(1600);
          } else if (!wasPlaying) {
            setPlaybackState(false);
          }
        } else {
          setPlaybackState(false);
        }
        return;
      }

      showError("SF2 Error!", "See serial log", "Choose another SF2");
      delay(1800);

      if (!oldPath.isEmpty()) {
        String oldTail = tailForUi(oldPath);
        midiCloseStream();
        resetChannels();
        showBusy("Restoring SF2...", oldTail.c_str());
        if (loadSF2(oldPath.c_str())) {
          if (hadMidi) {
            String midiTail = tailForUi(g_midiList[oldMidiIdx]);
            showBusy("Reloading MIDI...", midiTail.c_str());
            if (loadMidi(oldMidiIdx) && !wasPlaying) {
              setPlaybackState(false);
            }
          } else {
            setPlaybackState(false);
          }
        }
      }
    }

  } else if (choice == 3) {
    String dir = "/";
    { int sl = g_cfg.midiPath.lastIndexOf('/'); if (sl > 0) dir = g_cfg.midiPath.substring(0, sl); }
    String chosen = FileSelector::select({ ".mid",".midi",".MID",".MIDI" }, g_cfg.midiPath, "MIDI File");
    if (chosen.length() > 0) {
      g_cfg.midiPath = chosen;
      int sl = chosen.lastIndexOf('/');
      buildMidiList((sl > 0) ? chosen.substring(0, sl) : "/", chosen);
      saveCurrentMidiConfig(g_midiIdx);
      loadMidi(g_midiIdx);
      return; // loadMidi already restored g_playing = true
    }
  }

  g_playing    = wasPlaying;
  g_lastPlaybackActivityMs = millis();
  g_fullRedraw = true;
  g_needRedraw = true;
}

// ══════════════════════════════════════════════════════════════════════════════
// SETUP
// ══════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.print("\r\n[BOOT] GM MIDI Player - Cardputer ADV\r\n");

  // Display
  auto mcfg = M5.config();
  mcfg.internal_mic = false;
  mcfg.internal_spk = true;
  M5Cardputer.begin(mcfg, true);
  auto& d = M5Cardputer.Display;
  d.setRotation(1);
  d.fillScreen(C_BG);
  d.setTextFont(0);
  d.setTextSize(1);

  // SD card
  g_spiSD.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  if (!SD.begin(PIN_SD_CS, g_spiSD, 25000000UL)) {
    showError("SD Card Error!", "SCK:40 MISO:39 MOSI:14 CS:12");
    for (;;) delay(1000);
  }

  showBootSplashImage(2000);

  // Load saved config.
  g_cfg = loadConfig();
  // Select the audio mode, using the saved config as the default.

  g_audioMode      = selectAudioMode(g_cfg.audioMode);
  g_cfg.audioMode  = (int)g_audioMode;

  // Splash screen
  d.fillScreen((uint16_t)0x000A);
  d.setTextColor(C_TEXT); d.setTextSize(2);
  d.setCursor(14, 10); d.println("GM MIDI Player");
  d.setTextSize(1); d.setTextColor(C_DIM);
  d.setCursor(14, 35);
#ifndef CARDPUTER_V11
  d.println("Cardputer ADV");
#else
  d.println("Cardputer v1.1");
#endif
  d.drawFastHLine(0, 48, 240, C_ACCENT);
  int sy = 55;
  auto sl_ = [&](const char* msg) {
    d.setTextColor(C_DIM, (uint16_t)0x000A);
    d.fillRect(0, sy, 240, 10, (uint16_t)0x000A);
    d.setCursor(14, sy); d.println(msg); sy += 12;
    Serial.printf("[INIT] %s\r\n", msg);
  };

  // Initialize audio before loading the SF2 so g_sampleRate is correct.
  sl_("Init audio...");
  if (!initAudio()) {
    showError("Audio init failed!", "See serial log", "Cannot continue");
    for (;;) delay(1000);
  }

  // SF2 file browser
  showBusy("Select soundfont...");
  {
    String dir = parentDirOrRoot(g_cfg.sf2Path);
    String chosen = FileSelector::select({ ".sf2",".SF2" }, g_cfg.sf2Path, "GM Soundfont (.sf2)");
    if (chosen.length() > 0) g_cfg.sf2Path = chosen;
    if (g_cfg.sf2Path.isEmpty()) g_cfg.sf2Path = "/gm.sf2";
  }

  // Load SF2. Do not continue to MIDI selection until a valid soundfont is loaded.
  while (true) {
    String tail = tailForUi(g_cfg.sf2Path);
    showBusy("Loading SF2...", tail.c_str());
    if (loadSF2(g_cfg.sf2Path.c_str())) {
      showBusy("SF2 OK", tail.c_str());
      delay(500);
      break;
    }

    showError("SF2 Error!", "See serial log", "Choose another SF2");
    delay(1800);

    while (true) {
      String dir = parentDirOrRoot(g_cfg.sf2Path);
      String chosen = FileSelector::select({ ".sf2",".SF2" }, g_cfg.sf2Path, "GM Soundfont (.sf2)");
      if (chosen.length() > 0) {
        g_cfg.sf2Path = chosen;
        break;
      }

      showError("SF2 required", "Choose a valid SF2", "Cannot continue");
      delay(1400);
    }
  }

  // MIDI file browser
  showBusy("Select MIDI file...");
  {
    String dir = parentDirOrRoot(g_cfg.midiPath);
    String chosen = FileSelector::select({ ".mid",".midi",".MID",".MIDI" }, g_cfg.midiPath, "MIDI File (.mid)");
    if (chosen.length() > 0) g_cfg.midiPath = chosen;
    if (g_cfg.midiPath.isEmpty()) {
      showError("No MIDI file selected!");
      for (;;) delay(1000);
    }
  }

  // Build the MIDI list from the same folder.
  {
    int    sl     = g_cfg.midiPath.lastIndexOf('/');
    String folder = (sl > 0) ? g_cfg.midiPath.substring(0, sl) : "/";
    buildMidiList(folder, g_cfg.midiPath);
    g_cfg.midiIdx = g_midiIdx;
  }

  // Save the full config.
  saveConfig(g_cfg);
  showBusy("Config saved");
  delay(500);

  // Keep audio on the other core and run UI from Arduino loop().
  g_tsfMutex = xSemaphoreCreateMutex();
  if (!g_tsfMutex) {
    showError("Mutex init failed!", "Out of memory?", "Cannot continue");
    for (;;) delay(1000);
  }
  xTaskCreatePinnedToCore(audioTask, "Audio", AUDIO_TASK_STACK, nullptr,
                          AUDIO_TASK_PRIORITY, nullptr, 0);

  // Start playback
  delay(200);
  resetChannels();
  if (!loadMidi(g_midiIdx)) {
    showError("MIDI load failed!", "See serial log", "Cannot continue");
    for (;;) delay(1000);
  }

  g_fullRedraw  = true;
  g_lastTickUs  = micros();
  g_lastUiMs    = millis();
  g_lastPlaybackActivityMs = millis();
  invalidateUiCache();
  redrawFull();
  Serial.println("[INIT] UI ready, entering main loop");
  Serial.flush();
}

// ══════════════════════════════════════════════════════════════════════════════
// ES8311 AUDIO STEP  (runs on Core 1, same as M5Cardputer.update)
// ══════════════════════════════════════════════════════════════════════════════

static void es8311AudioStep() {
  if (!g_audioReady || !g_tsf) return;

  if (!g_playing) {
    resetEs8311Buffers(true);
    return;
  }

  es8311ReclaimConsumedBuffers();

  while (g_es8311QueueCount < 2) {
    int readyIdx = es8311FindBuffer(ES8311_BUF_READY);
    if (readyIdx < 0) break;

    uint32_t playStartUs = micros();
    bool ok = M5Cardputer.Speaker.playRaw(g_es8311Buf[readyIdx], ES8311_CHUNK_FRAMES, g_sampleRate, false, 1, 0, false);
    uint32_t playUs = micros() - playStartUs;

    if (g_perf.startedMs == 0) perfReset(millis());
    ++g_perf.playCalls;
    g_perf.playUsTotal += playUs;
    if (playUs > g_perf.playUsMax) g_perf.playUsMax = playUs;

    if (!ok) {
      ++g_perf.playFail;
      break;
    }

    ++g_perf.playOk;
    g_es8311BufState[readyIdx] = ES8311_BUF_QUEUED;
    g_es8311QueueOrder[(g_es8311QueueHead + g_es8311QueueCount) % ES8311_QUEUE_BUFFERS] = (uint8_t)readyIdx;
    ++g_es8311QueueCount;
  }

  int emptyIdx = es8311FindBuffer(ES8311_BUF_EMPTY);
  if (emptyIdx >= 0) {
    uint32_t stepStartUs = micros();
    Es8311RenderStats renderStats = renderEs8311Chunk(g_es8311Buf[emptyIdx], ES8311_CHUNK_FRAMES);
    uint32_t stepUs = micros() - stepStartUs;
    uint32_t chunkBudgetUs = (g_sampleRate > 0)
                           ? (uint32_t)((1000000ULL * ES8311_CHUNK_FRAMES) / g_sampleRate)
                           : 0;

    g_es8311BufState[emptyIdx] = ES8311_BUF_READY;

    if (g_perf.startedMs == 0) perfReset(millis());
    ++g_perf.chunkCalls;
    g_perf.chunkUsTotal += stepUs;
    if (stepUs > g_perf.chunkUsMax) g_perf.chunkUsMax = stepUs;
    if (chunkBudgetUs && stepUs > chunkBudgetUs) ++g_perf.lateChunks;
    g_perf.renderUsTotal += renderStats.eventUs + renderStats.synthUs;
    if (renderStats.eventUs + renderStats.synthUs > g_perf.renderUsMax) {
      g_perf.renderUsMax = renderStats.eventUs + renderStats.synthUs;
    }
    g_perf.eventUsTotal += renderStats.eventUs;
    if (renderStats.eventUs > g_perf.eventUsMax) g_perf.eventUsMax = renderStats.eventUs;
    g_perf.synthUsTotal += renderStats.synthUs;
    if (renderStats.synthUs > g_perf.synthUsMax) g_perf.synthUsMax = renderStats.synthUs;
    g_perf.midiEvents += renderStats.midiEvents;
    g_perf.synthSegments += renderStats.synthSegments;
  }

  while (g_es8311QueueCount < 2) {
    int readyIdx = es8311FindBuffer(ES8311_BUF_READY);
    if (readyIdx < 0) break;

    uint32_t playStartUs = micros();
    bool ok = M5Cardputer.Speaker.playRaw(g_es8311Buf[readyIdx], ES8311_CHUNK_FRAMES, g_sampleRate, false, 1, 0, false);
    uint32_t playUs = micros() - playStartUs;

    if (g_perf.startedMs == 0) perfReset(millis());
    ++g_perf.playCalls;
    g_perf.playUsTotal += playUs;
    if (playUs > g_perf.playUsMax) g_perf.playUsMax = playUs;

    if (!ok) {
      ++g_perf.playFail;
      break;
    }

    ++g_perf.playOk;
    g_es8311BufState[readyIdx] = ES8311_BUF_QUEUED;
    g_es8311QueueOrder[(g_es8311QueueHead + g_es8311QueueCount) % ES8311_QUEUE_BUFFERS] = (uint8_t)readyIdx;
    ++g_es8311QueueCount;
  }
}

// ══════════════════════════════════════════════════════════════════════════════
// LOOP
// ══════════════════════════════════════════════════════════════════════════════

void loop() {
  uint32_t uiStartUs = micros();
  playerUiStep();
  uint32_t uiUs = micros() - uiStartUs;
  if (g_perf.startedMs == 0) perfReset(millis());
  ++g_perf.uiCalls;
  g_perf.uiUsTotal += uiUs;
  if (uiUs > g_perf.uiUsMax) g_perf.uiUsMax = uiUs;

  if (g_audioMode == AUDIO_ES8311) es8311AudioStep();
  perfMaybeLog();
  delay(1);
}
