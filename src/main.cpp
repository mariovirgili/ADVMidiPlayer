/*
 * ══════════════════════════════════════════════════════════════════════════════
 *  GM MIDI Player  —  M5Stack Cardputer ADV
 *  PlatformIO / Arduino framework
 * ══════════════════════════════════════════════════════════════════════════════
 *
 *  Modalità audio (menu interattivo al boot):
 *    1 → ES8311  codec integrato ADV  — jack 3.5mm + speaker, 44100Hz mono
 *    2 → I2S DAC esterno              — MAX98357A/PCM5102,    22050Hz stereo
 *    3 → PDM speaker GPIO2            — nessun HW extra,      16000Hz mono
 *    4 → PWM LEDC GPIO2              — fallback,             16000Hz mono
 *
 *  (-DCARDPUTER_V11 → esclude opzione ES8311, rinumera 1-3)
 *
 *  Player controls:
 *    SPACE       Play / Pause
 *    ENTER / /   Next track
 *    ,           Previous track
 *    R           Restart current track
 *    L           Toggle loop
 *    + / -       Volume +3 / -3 dB
 *    F           Open the file selector (SF2 or MIDI) without rebooting
 *
 *  Configuration stored on SD: /midi_player.cfg
 * ══════════════════════════════════════════════════════════════════════════════
 */

/*
 * English summary:
 * - Audio output is selected from an interactive boot menu.
 * - Player navigation uses ; . , / as up / down / left / right shortcuts.
 * - The config file is stored on the SD card at /midi_player.cfg.
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

#define CHUNK_FRAMES  256
#define I2S_DMA_BUFS  6
#define I2S_DMA_LEN   256
#define MAX_VOICES    16

static int16_t g_audioBuf[CHUNK_FRAMES];
static int16_t g_i2sBuf [CHUNK_FRAMES * 2];

static volatile uint32_t g_pwmIdx  = 0;
static volatile bool     g_pwmDone = false;
static hw_timer_t*       g_pwmTimer = nullptr;

static inline bool useStereoSynthOutput() {
  return g_audioMode == AUDIO_I2S_DAC;
}

static inline enum TSFOutputMode currentSynthOutputMode() {
  return useStereoSynthOutput() ? TSF_STEREO_INTERLEAVED : TSF_MONO;
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

// ══════════════════════════════════════════════════════════════════════════════
// GLOBALS
// ══════════════════════════════════════════════════════════════════════════════

static tsf*              g_tsf      = nullptr;
static SemaphoreHandle_t g_tsfMutex = nullptr;

static tml_message* g_midiRoot = nullptr;
static tml_message* g_midiCur  = nullptr;
static double       g_midiMs   = 0.0;

static std::vector<String> g_midiList;
static int          g_midiIdx  = 0;
static bool         g_playing  = false;
static bool         g_looping  = false;
static float        g_vol_dB   = -18.0f;

static ChannelInfo  g_ch[16];
static bool         g_fullRedraw = true;
static bool         g_needRedraw = true;
static uint32_t     g_lastUiMs   = 0;
static uint32_t     g_lastTickMs = 0;
static uint32_t     g_songDurMs  = 0;

static PlayerConfig g_cfg;
static File         g_sf2File;
static File         g_midiFile;

// ══════════════════════════════════════════════════════════════════════════════
// SD STREAM ADAPTERS
// ══════════════════════════════════════════════════════════════════════════════

static int sf2_read(void*, void* p, unsigned s) { return (int)g_sf2File.read((uint8_t*)p, s); }
static int sf2_skip(void*, unsigned n)          { return g_sf2File.seek(g_sf2File.position() + n) ? 1 : 0; }

static int tml_read(void*, void* p, unsigned s) { return (int)g_midiFile.read((uint8_t*)p, s); }

// ══════════════════════════════════════════════════════════════════════════════
// AUDIO INIT
// ══════════════════════════════════════════════════════════════════════════════

static void initAudio() {
  switch (g_audioMode) {

    case AUDIO_ES8311: {
      // ES8311 codec built into the Cardputer ADV.
      // M5.Speaker.begin() configures ES8311 over I2C and starts I2S_NUM_0 at 44100Hz.
      // After begin() we write directly to I2S instead of going through M5.Speaker.
      g_sampleRate = 44100;
      M5.Speaker.setVolume(200);
      M5.Speaker.begin();
      Serial.printf("[AUDIO] ES8311  %uHz mono\n", g_sampleRate);
      break;
    }

    case AUDIO_I2S_DAC: {
      g_sampleRate = 22050;
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
      i2s_driver_install(I2S_NUM_0, &c, 0, nullptr);
      i2s_pin_config_t p{};
      p.bck_io_num    = PIN_I2S_BCK;
      p.ws_io_num     = PIN_I2S_WS;
      p.data_out_num  = PIN_I2S_DOUT;
      p.data_in_num   = I2S_PIN_NO_CHANGE;
      i2s_set_pin(I2S_NUM_0, &p);
      Serial.printf("[AUDIO] I2S DAC  %uHz stereo\n", g_sampleRate);
      break;
    }

    case AUDIO_PDM: {
      g_sampleRate = 16000;
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
      i2s_driver_install(I2S_NUM_0, &c, 0, nullptr);
      i2s_pin_config_t p{};
      p.bck_io_num    = PIN_PDM_CLK;
      p.ws_io_num     = I2S_PIN_NO_CHANGE;
      p.data_out_num  = PIN_SPEAKER;
      p.data_in_num   = I2S_PIN_NO_CHANGE;
      i2s_set_pin(I2S_NUM_0, &p);
      Serial.printf("[AUDIO] PDM GPIO%d  %uHz mono\n", PIN_SPEAKER, g_sampleRate);
      break;
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
      Serial.printf("[AUDIO] PWM GPIO%d  %uHz mono\n", PIN_SPEAKER, g_sampleRate);
      break;
    }

    default: break;
  }
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
    if (g_tsf && g_playing) {
      xSemaphoreTake(g_tsfMutex, portMAX_DELAY);
      if (useStereoSynthOutput()) {
        tsf_render_short(g_tsf, g_i2sBuf, CHUNK_FRAMES, 0);
      } else {
        tsf_render_short(g_tsf, g_audioBuf, CHUNK_FRAMES, 0);
      }
      xSemaphoreGive(g_tsfMutex);
    } else {
      memset(g_audioBuf, 0, sizeof(g_audioBuf));
      memset(g_i2sBuf, 0, sizeof(g_i2sBuf));
    }

    switch (g_audioMode) {
      case AUDIO_ES8311:
        monoToStereoDup(g_audioBuf, g_i2sBuf, CHUNK_FRAMES);
        i2s_write(I2S_NUM_0, g_i2sBuf, CHUNK_FRAMES * 4, &written, portMAX_DELAY);
        break;
      case AUDIO_I2S_DAC:
        i2s_write(I2S_NUM_0, g_i2sBuf, CHUNK_FRAMES * 4, &written, portMAX_DELAY);
        break;
      case AUDIO_PDM:
        i2s_write(I2S_NUM_0, g_audioBuf, CHUNK_FRAMES * 2, &written, portMAX_DELAY);
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

// ══════════════════════════════════════════════════════════════════════════════
// SF2 LOAD
// ══════════════════════════════════════════════════════════════════════════════

static bool loadSF2(const char* path) {
  if (g_tsf) { tsf_close(g_tsf); g_tsf = nullptr; }
  Serial.printf("[SF2] %s  heap=%u\n", path, heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
  g_sf2File = SD.open(path);
  if (!g_sf2File) { Serial.println("[SF2] File non trovato"); return false; }
  tsf_stream s{ nullptr, sf2_read, sf2_skip };
  g_tsf = tsf_load(&s);
  g_sf2File.close();
  if (!g_tsf) { Serial.println("[SF2] tsf_load fallito"); return false; }
  tsf_set_output(g_tsf, currentSynthOutputMode(), (int)g_sampleRate, g_vol_dB);
  tsf_set_max_voices(g_tsf, MAX_VOICES);
  Serial.printf("[SF2] OK  preset=%d  heap=%u\n",
    tsf_get_presetcount(g_tsf), heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
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
}

// Scansiona cartella → g_midiList; imposta g_midiIdx sul file target
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
  if (g_tsf) {
    xSemaphoreTake(g_tsfMutex, portMAX_DELAY);
    tsf_note_off_all(g_tsf);
    xSemaphoreGive(g_tsfMutex);
  }
  delay(60);

  if (g_midiRoot) { tml_free(g_midiRoot); g_midiRoot = nullptr; }
  resetChannels();

  g_midiFile = SD.open(g_midiList[idx].c_str());
  if (!g_midiFile) { Serial.println("[MIDI] File non trovato"); return false; }
  tml_stream ts{ nullptr, tml_read };
  g_midiRoot = tml_load(&ts);
  g_midiFile.close();
  if (!g_midiRoot) { Serial.println("[MIDI] tml_load fallito"); return false; }

  tml_message* last = g_midiRoot;
  while (last->next) last = last->next;
  g_songDurMs = (uint32_t)last->time;

  g_midiCur   = g_midiRoot;
  g_midiMs    = 0.0;
  g_midiIdx   = idx;
  g_playing   = true;
  g_fullRedraw = true;
  Serial.printf("[MIDI] %s  dur=%us\n", g_midiList[idx].c_str(), g_songDurMs / 1000);
  return true;
}

// ══════════════════════════════════════════════════════════════════════════════
// MIDI TICK
// ══════════════════════════════════════════════════════════════════════════════

static void tickMidi(uint32_t deltaMs) {
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
        g_needRedraw = true;
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
        g_needRedraw = true;
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
      g_needRedraw      = true;
    }
  }

  // Fine brano → avanza automaticamente
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

  String fname = "";
  if (!g_midiList.empty()) {
    fname = g_midiList[g_midiIdx];
    int sl = fname.lastIndexOf('/'); if (sl >= 0) fname = fname.substring(sl + 1);
    int dt = fname.lastIndexOf('.'); if (dt > 0)  fname = fname.substring(0, dt);
    if ((int)fname.length() > 16) fname = fname.substring(0, 15) + "~";
  }
  d.setCursor(2, 2);
  d.print(g_playing ? "\x10 " : "|| ");
  d.print(g_looping ? "[L] " : "    ");
  d.print(fname);

  char info[26];
  uint32_t el  = (uint32_t)(g_midiMs / 1000);
  uint32_t tot = g_songDurMs / 1000;
  snprintf(info, sizeof(info), "%2d/%d %u:%02u/%u:%02u",
           g_midiIdx+1, (int)g_midiList.size(),
           el/60, el%60, tot/60, tot%60);
  d.setCursor(240 - (int)strlen(info)*6 - 2, 2);
  d.print(info);
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
  d.print("SPC:play  ,/:trk  ;.:vol  L:loop  F:file");
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
  M5Cardputer.Display.fillScreen(C_BG);
  drawHeader();
  for (int i = 0; i < 16; i++) drawChannelRow(i);
  drawProgressBar();
  g_fullRedraw = false;
  g_needRedraw = false;
}

static void redrawPartial() {
  drawHeader();
  for (int i = 0; i < 16; i++) drawChannelRow(i);
  drawProgressBar();
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
      "44kHz mono - built-in output",
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

// ══════════════════════════════════════════════════════════════════════════════
// SELETTORE FILE A RUNTIME  (tasto F)
// ══════════════════════════════════════════════════════════════════════════════

static void openFileSelector() {
  // Runtime file selector opened with the F key.
  bool wasPlaying = g_playing;
  g_playing = false;
  if (g_tsf) {
    xSemaphoreTake(g_tsfMutex, portMAX_DELAY);
    tsf_note_off_all(g_tsf);
    xSemaphoreGive(g_tsfMutex);
  }
  delay(60);

  auto& d = M5Cardputer.Display;

  // Mini menu: SF2 / MIDI / Cancel
  int sel = 0;
  auto drawMenu = [&]() {
    d.fillScreen((uint16_t)0x000A);
    d.setTextColor(C_ACCENT, (uint16_t)0x000A); d.setTextSize(1);
    d.setCursor(4, 4); d.print("Change file:");
    d.drawFastHLine(0, 14, 240, C_ACCENT);
    struct { const char* k; const char* l; uint16_t c; } opts[] = {
      { "1", "GM Soundfont (.sf2)", 0x07E0 },
      { "2", "MIDI File   (.mid)", 0x07FF },
      { "3", "Cancel",            0x8410 },
    };
    for (int i = 0; i < 3; i++) {
      int y = 22 + i * 34;
      uint16_t bg = (i == sel) ? (uint16_t)0x0318 : (uint16_t)0x1082;
      d.fillRect(4, y, 232, 28, bg);
      d.drawRoundRect(4, y, 232, 28, 3, (i == sel) ? opts[i].c : (uint16_t)0x4208);
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
    }
    if (!choice && (cardputer_keyboard::pressed_word_ci(ks, prevKeys, 'w') ||
                    cardputer_keyboard::pressed_nav_up(ks, prevKeys))) {
      sel = (sel + 2) % 3;
      drawMenu();
    }
    if (!choice && (cardputer_keyboard::pressed_word_ci(ks, prevKeys, 's') ||
                    cardputer_keyboard::pressed_nav_down(ks, prevKeys))) {
      sel = (sel + 1) % 3;
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
    String dir = "/";
    { int sl = g_cfg.sf2Path.lastIndexOf('/'); if (sl > 0) dir = g_cfg.sf2Path.substring(0, sl); }
    String chosen = FileSelector::select({ ".sf2", ".SF2" }, dir, "GM Soundfont");
    if (chosen.length() > 0) {
      showBusy("Loading SF2...",
               chosen.substring(chosen.length() > 28 ? chosen.length()-28 : 0).c_str());
      if (loadSF2(chosen.c_str())) {
        g_cfg.sf2Path = chosen; saveConfig(g_cfg);
      } else {
        showError("SF2 Error!", "Invalid or oversized file", "Restoring...");
        delay(2000);
        loadSF2(g_cfg.sf2Path.c_str());
      }
    }

  } else if (choice == 2) {
    String dir = "/";
    { int sl = g_cfg.midiPath.lastIndexOf('/'); if (sl > 0) dir = g_cfg.midiPath.substring(0, sl); }
    String chosen = FileSelector::select({ ".mid",".midi",".MID",".MIDI" }, dir, "MIDI File");
    if (chosen.length() > 0) {
      g_cfg.midiPath = chosen;
      int sl = chosen.lastIndexOf('/');
      buildMidiList((sl > 0) ? chosen.substring(0, sl) : "/", chosen);
      g_cfg.midiIdx = g_midiIdx;
      saveConfig(g_cfg);
      loadMidi(g_midiIdx);
      return; // loadMidi already restored g_playing = true
    }
  }

  g_playing    = wasPlaying;
  g_fullRedraw = true;
}

// ══════════════════════════════════════════════════════════════════════════════
// SETUP
// ══════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[BOOT] GM MIDI Player — Cardputer ADV");

  // Display
  auto mcfg = M5.config();
  M5Cardputer.begin(mcfg, true);
  auto& d = M5Cardputer.Display;
  d.setRotation(1);
  d.fillScreen(C_BG);
  d.setTextFont(0);
  d.setTextSize(1);

  // SD card
  SPIClass spiSD(HSPI);
  spiSD.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  if (!SD.begin(PIN_SD_CS, spiSD, 25000000UL)) {
    showError("SD Card Error!", "SCK:40 MISO:39 MOSI:14 CS:12");
    for (;;) delay(1000);
  }

  // Load saved config.
  g_cfg = loadConfig();
  // Select the audio mode, using the saved config as the default.

  // Selezione modalità audio (con default dalla config)
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
    Serial.printf("[INIT] %s\n", msg);
  };

  // Initialize audio before loading the SF2 so g_sampleRate is correct.
  sl_("Init audio...");
  initAudio();

  // SF2 file browser
  sl_("Select soundfont...");
  {
    String dir = "/";
    { int sl = g_cfg.sf2Path.lastIndexOf('/'); if (sl > 0) dir = g_cfg.sf2Path.substring(0, sl); }
    String chosen = FileSelector::select({ ".sf2",".SF2" }, dir, "GM Soundfont (.sf2)");
    if (chosen.length() > 0) g_cfg.sf2Path = chosen;
    if (g_cfg.sf2Path.isEmpty()) g_cfg.sf2Path = "/gm.sf2";
  }

  // Load SF2
  sl_("Loading SF2...");
  d.setTextColor((uint16_t)0x4208, (uint16_t)0x000A);
  d.setCursor(14, sy); d.print(g_cfg.sf2Path.c_str()); sy += 12;
  if (!loadSF2(g_cfg.sf2Path.c_str())) {
    showError("SF2 Error!", g_cfg.sf2Path.c_str(), "Invalid or oversized file");
    delay(3000);
  } else {
    sl_("SF2 OK");
  }

  // MIDI file browser
  sl_("Select MIDI file...");
  {
    String dir = "/";
    { int sl = g_cfg.midiPath.lastIndexOf('/'); if (sl > 0) dir = g_cfg.midiPath.substring(0, sl); }
    String chosen = FileSelector::select({ ".mid",".midi",".MID",".MIDI" }, dir, "MIDI File (.mid)");
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
  sl_("Config saved");

  // Audio task on Core 0.
  g_tsfMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(audioTask, "Audio", 6144, nullptr,
                          configMAX_PRIORITIES - 1, nullptr, 0);

  // Start playback
  delay(200);
  resetChannels();
  loadMidi(g_midiIdx);

  g_fullRedraw  = true;
  g_lastTickMs  = millis();
  g_lastUiMs    = millis();
}

// ══════════════════════════════════════════════════════════════════════════════
// LOOP
// ══════════════════════════════════════════════════════════════════════════════

void loop() {
  static cardputer_keyboard::KeysState prevKeys{};
  M5Cardputer.update();

  uint32_t now   = millis();
  uint32_t delta = now - g_lastTickMs;
  g_lastTickMs   = now;
  if (delta > 0 && delta < 200) tickMidi(delta);

  // Keyboard input
  if (M5Cardputer.Keyboard.isChange()) {
    auto ks = M5Cardputer.Keyboard.keysState();

    // SPACE → play/pause
    // SPACE -> play/pause
    if (cardputer_keyboard::pressed_space(ks, prevKeys)) {
      g_playing = !g_playing;
      if (!g_playing && g_tsf) {
        xSemaphoreTake(g_tsfMutex, portMAX_DELAY);
        tsf_note_off_all(g_tsf);
        xSemaphoreGive(g_tsfMutex);
      }
      g_fullRedraw = true;
    }

    for (char c : ks.word) {
      // Next track
      if (c == '\n' || c == '\r' || c == '/' || c == '?') {
        g_midiIdx = (g_midiIdx + 1) % (int)g_midiList.size();
        g_cfg.midiIdx = g_midiIdx; saveConfig(g_cfg);
        loadMidi(g_midiIdx); break;
      }
      // Previous track
      if (c == ',' || c == '<') {
        g_midiIdx = (g_midiIdx - 1 + (int)g_midiList.size()) % (int)g_midiList.size();
        g_cfg.midiIdx = g_midiIdx; saveConfig(g_cfg);
        loadMidi(g_midiIdx); break;
      }
      // Restart current track
      if (c == 'r' || c == 'R') { loadMidi(g_midiIdx); break; }
      // Loop
      if (c == 'l' || c == 'L') { g_looping = !g_looping; g_needRedraw = true; }
      // Volume up
      if (c == ';' || c == ':' || c == '+' || c == '=') {
        g_vol_dB = (g_vol_dB + 3.0f < 0.0f) ? g_vol_dB + 3.0f : 0.0f;
        if (g_tsf) {
          xSemaphoreTake(g_tsfMutex, portMAX_DELAY);
          tsf_set_output(g_tsf, currentSynthOutputMode(), (int)g_sampleRate, g_vol_dB);
          xSemaphoreGive(g_tsfMutex);
        }
        Serial.printf("[VOL] %.0f dB\n", g_vol_dB);
      }
      // Volume giù
      // Volume down
      if (c == '.' || c == '>' || c == '-') {
        g_vol_dB = (g_vol_dB - 3.0f > -40.0f) ? g_vol_dB - 3.0f : -40.0f;
        if (g_tsf) {
          xSemaphoreTake(g_tsfMutex, portMAX_DELAY);
          tsf_set_output(g_tsf, currentSynthOutputMode(), (int)g_sampleRate, g_vol_dB);
          xSemaphoreGive(g_tsfMutex);
        }
        Serial.printf("[VOL] %.0f dB\n", g_vol_dB);
      }
      // File selector
      if (c == 'f' || c == 'F') { openFileSelector(); break; }
    }
    prevKeys = ks;
  }

  // Refresh UI ~15 fps
  if (now - g_lastUiMs >= 66) {
    g_lastUiMs = now;
    if (g_fullRedraw)       redrawFull();
    else if (g_needRedraw)  redrawPartial();
  }

  delay(1);
}
