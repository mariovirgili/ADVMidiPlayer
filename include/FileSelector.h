#pragma once
/*
 * FileSelector.h — Browser file SD per M5Stack Cardputer
 *
 * Controls:
 *   W / S        → su / giù
 *   ENTER        → apri cartella / seleziona file
 *   BKSP (←)    → cartella superiore
 *   ESC / `      → annulla → ritorna ""
 *   a-z          → salta alla prima voce che inizia con quella lettera
 *
 * Config salvata su SD in /Midi/midi_player.cfg:
 *   sf2=/path/file.sf2
 *   midi=/path/file.mid
 *   displaymode=0
 *   audiomode=1
 *   mididx=0
 */

/*
 * English summary:
 * - SD file browser for the Cardputer.
 * - ; and . move up/down, , and / move by one full page, Enter opens/selects.
 * - Backspace goes to the parent folder. Fn+` cancels and returns an empty string.
 */

#include <M5Cardputer.h>
#include <SD.h>
#include <vector>
#include <algorithm>

#include "CardputerKeyboard.h"

#define CFG_DIR "/Midi"
#define CFG_PATH "/Midi/midi_player.cfg"
#define CFG_PATH_LEGACY "/midi_player.cfg"

// ── Palette ──────────────────────────────────────────────────────────────────
#define FS_BG       0x0000u
#define FS_HDR_BG   0x0318u
#define FS_SEL_BG   0x2124u
#define FS_SEL_BRD  0x07FFu
#define FS_DIR_COL  0xFFE0u
#define FS_DIM      0x8410u
#define FS_FOOT_BG  0x18C3u
#define FS_FOOT_TXT 0x7BEFu
#define FS_SCR_FG   0x07FFu
#define FS_SCR_BG   0x2104u

// ─────────────────────────────────────────────────────────────────────────────

struct FsEntry {
    String name;
    bool   isDir;
    bool operator<(const FsEntry& o) const {
        if (isDir != o.isDir) return isDir > o.isDir;
        String a = name; a.toLowerCase();
        String b = o.name; b.toLowerCase();
        return a < b;
    }
};

// ─────────────────────────────────────────────────────────────────────────────

class FileSelector {
public:
    static const int W       = 240;
    static const int H       = 135;
    static const int HDR_H   = 13;
    static const int FOOT_H  = 11;
    static const int ROW_H   = 11;
    static const int LIST_Y  = HDR_H;
    static const int LIST_HEIGHT = H - HDR_H - FOOT_H;
    static const int VISIBLE_ROWS = LIST_HEIGHT / ROW_H;   // 10
    static const int SCRW    = 4;

    // Returns the full selected path, or "" if cancelled.
    static String select(const std::vector<String>& ext,
                         const String& start = "/",
                         const char*   title = "Select")
    {
        String curDir = "/";
        String initialName;
        bool applyInitialSelection = false;
        int cursor = 0, scroll = 0;
        std::vector<FsEntry> entries;
        bool redraw = true;

        auto applyStartPath = [&]() {
            String startPath = start.isEmpty() ? "/" : start;
            File probe = SD.open(startPath.c_str());
            if (probe) {
                if (probe.isDirectory()) {
                    curDir = startPath;
                } else {
                    initialName = String(probe.name());
                    int slash = startPath.lastIndexOf('/');
                    curDir = (slash > 0) ? startPath.substring(0, slash) : "/";
                    applyInitialSelection = initialName.length() > 0;
                }
                probe.close();
                return;
            }

            int slash = startPath.lastIndexOf('/');
            if (slash >= 0) {
                String tail = startPath.substring(slash + 1);
                if (tail.indexOf('.') >= 0) {
                    initialName = tail;
                    curDir = (slash > 0) ? startPath.substring(0, slash) : "/";
                    applyInitialSelection = initialName.length() > 0;
                    return;
                }
            }
            curDir = startPath;
        };

        auto loadDir = [&]() {
            entries.clear(); cursor = 0; scroll = 0;
            File dir = SD.open(curDir.c_str());
            if (!dir || !dir.isDirectory()) return;
            File f;
            while ((f = dir.openNextFile())) {
                FsEntry e;
                e.name  = String(f.name());
                e.isDir = f.isDirectory();
                if (!e.isDir) {
                    bool ok = false;
                    String nl = e.name; nl.toLowerCase();
                    for (const auto& x : ext) {
                        String xl = x; xl.toLowerCase();
                        if (nl.endsWith(xl)) { ok = true; break; }
                    }
                    if (!ok) { f.close(); continue; }
                }
                if (!e.name.startsWith(".")) entries.push_back(e);
                f.close();
            }
            dir.close();
            std::sort(entries.begin(), entries.end());

            if (applyInitialSelection && initialName.length() > 0) {
                for (int i = 0; i < (int)entries.size(); ++i) {
                    if (entries[i].name.equalsIgnoreCase(initialName)) {
                        cursor = i;
                        scroll = (cursor > VISIBLE_ROWS / 2) ? cursor - VISIBLE_ROWS / 2 : 0;
                        int maxScroll = std::max(0, (int)entries.size() - VISIBLE_ROWS);
                        if (scroll > maxScroll) scroll = maxScroll;
                        break;
                    }
                }
            }
            applyInitialSelection = false;
        };

        applyStartPath();
        loadDir();
        cardputer_keyboard::KeysState prevKeys{};

        while (true) {
            if (redraw) { _draw(entries, curDir, title, cursor, scroll); redraw = false; }
            M5Cardputer.update();

            auto ks = M5Cardputer.Keyboard.keysState();
            bool kbChanged = M5Cardputer.Keyboard.isChange()
                          || cardputer_keyboard::state_changed(ks, prevKeys);
            if (!kbChanged) { delay(5); continue; }

            if (cardputer_keyboard::pressed_escape(ks, prevKeys)) return "";

            if (cardputer_keyboard::pressed_backspace(ks, prevKeys)) {
                int sl = curDir.lastIndexOf('/');
                curDir = (sl > 0) ? curDir.substring(0, sl) : "/";
                loadDir();
                redraw = true;
            }

            if (cardputer_keyboard::pressed_enter(ks, prevKeys)) {
                if (!entries.empty()) {
                    const FsEntry& sel = entries[cursor];
                    if (sel.isDir) {
                        curDir = (curDir == "/") ? "/" + sel.name
                                                 : curDir + "/" + sel.name;
                        loadDir();
                    } else {
                        return (curDir == "/") ? "/" + sel.name
                                               : curDir + "/" + sel.name;
                    }
                    redraw = true;
                }
            }

            if (cardputer_keyboard::pressed_nav_left(ks, prevKeys)) {
                _pageMove(-1, (int)entries.size(), cursor, scroll);
                redraw = true;
            }

            if (cardputer_keyboard::pressed_nav_right(ks, prevKeys)) {
                _pageMove(+1, (int)entries.size(), cursor, scroll);
                redraw = true;
            }

            if (cardputer_keyboard::pressed_word_ci(ks, prevKeys, 'w') ||
                cardputer_keyboard::pressed_nav_up(ks, prevKeys)) {
                if (cursor > 0) { cursor--; if (cursor < scroll) scroll = cursor; }
                redraw = true;
            }

            if (cardputer_keyboard::pressed_word_ci(ks, prevKeys, 's') ||
                cardputer_keyboard::pressed_nav_down(ks, prevKeys)) {
                if (cursor < (int)entries.size() - 1) {
                    cursor++;
                    if (cursor >= scroll + VISIBLE_ROWS) scroll = cursor - VISIBLE_ROWS + 1;
                }
                redraw = true;
            }

            if (!ks.fn) {
                for (char c : ks.word) {
                    char lc = cardputer_keyboard::to_lower(c);
                    if (lc < 'a' || lc > 'z' || lc == 'w' || lc == 's') continue;
                    if (cardputer_keyboard::has_word_ci(prevKeys, lc)) continue;
                    for (int i = 0; i < (int)entries.size(); i++) {
                        String n = entries[i].name; n.toLowerCase();
                        if (!n.isEmpty() && n[0] == lc) {
                            cursor = i;
                            scroll = (cursor > VISIBLE_ROWS / 2) ? cursor - VISIBLE_ROWS / 2 : 0;
                            break;
                        }
                    }
                    redraw = true;
                }
            }

            prevKeys = ks;
            delay(5);
        }
    }

private:
    static void _pageMove(int dir, int count, int& cursor, int& scroll)
    {
        if (count <= 0) return;
        int maxCursor = count - 1;
        cursor += dir * VISIBLE_ROWS;
        if (cursor < 0) cursor = 0;
        if (cursor > maxCursor) cursor = maxCursor;
        scroll = (cursor / VISIBLE_ROWS) * VISIBLE_ROWS;
        int maxScroll = std::max(0, count - VISIBLE_ROWS);
        if (scroll > maxScroll) scroll = maxScroll;
    }

    static void _draw(const std::vector<FsEntry>& e, const String& dir,
                      const char* title, int cur, int scr)
    {
        auto& d = M5Cardputer.Display;

        // Header
        d.fillRect(0, 0, W, HDR_H, FS_HDR_BG);
        d.setTextColor(0xFFFFu, FS_HDR_BG); d.setTextSize(1);
        String hdr = String(title) + "  ";
        String p = dir;
        if (p.length() > 18) p = "~" + p.substring(p.length()-17);
        hdr += p;
        if (hdr.length() > 33) hdr = hdr.substring(0, 33);
        d.setCursor(3, 2); d.print(hdr);
        char cnt[12]; snprintf(cnt, 12, "%d items", (int)e.size());
        d.setCursor(W - (int)strlen(cnt)*6 - 2, 2); d.print(cnt);

        // File list
        d.fillRect(0, LIST_Y, W-SCRW, LIST_HEIGHT, FS_BG);
        for (int i = 0; i < VISIBLE_ROWS; i++) {
            int idx = scr + i, y = LIST_Y + i*ROW_H;
            if (idx >= (int)e.size()) break;
            bool sel = (idx == cur);
            if (sel) {
                d.fillRect(0, y, W-SCRW-1, ROW_H, FS_SEL_BG);
                d.drawRoundRect(0, y, W-SCRW-1, ROW_H, 2, FS_SEL_BRD);
            }
            uint16_t col = sel ? (e[idx].isDir ? (uint16_t)FS_DIR_COL : 0xFFFFu)
                               : (e[idx].isDir ? 0xC600u : (uint16_t)FS_DIM);
            d.setTextColor(col, sel ? (uint16_t)FS_SEL_BG : (uint16_t)FS_BG);
            d.setTextSize(1);
            d.setCursor(3, y+2); d.print(e[idx].isDir ? "\x07 " : "  ");
            String nm = e[idx].name;
            int mx = (W-SCRW-22)/6;
            if ((int)nm.length() > mx) nm = nm.substring(0, mx-1) + "~";
            d.setCursor(15, y+2); d.print(nm);
            if (e[idx].isDir && !sel) {
                d.setTextColor(0x4208u, (uint16_t)FS_BG);
                d.setCursor(W-SCRW-28, y+2); d.print("[DIR]");
            }
        }

        // Scrollbar
        d.fillRect(W-SCRW, LIST_Y, SCRW, LIST_HEIGHT, FS_SCR_BG);
        if (!e.empty() && (int)e.size() > VISIBLE_ROWS) {
            int sh = (LIST_HEIGHT * VISIBLE_ROWS) / (int)e.size();
            if (sh < 4) sh = 4;
            int sy2 = LIST_Y + (LIST_HEIGHT - sh) * scr / ((int)e.size() - VISIBLE_ROWS);
            d.fillRect(W-SCRW, sy2, SCRW, sh, FS_SCR_FG);
        }

        // Footer
        d.fillRect(0, H-FOOT_H, W, FOOT_H, FS_FOOT_BG);
        d.setTextColor(FS_FOOT_TXT, FS_FOOT_BG);
        d.setCursor(2, H-FOOT_H+2);
        d.print(";.:nav ,/:pg ENT:open BKSP:up ESC:x");
    }
};

// ═════════════════════════════════════════════════════════════════════════════
// CONFIG — read/write /Midi/midi_player.cfg
// ═════════════════════════════════════════════════════════════════════════════

struct PlayerConfig {
    // Config read/write for /Midi/midi_player.cfg
    String sf2Path   = "/gm.sf2";
    String midiPath  = "";
    int    displayMode = 0; // 0=single internal display, 1=dual display
    int    audioMode = 1;   // 1=ES8311 2=I2S_DAC 3=PDM 4=PWM
    int    midiIdx   = 0;
};

static inline String _cfgVal(const String& line, const String& key) {
    if (!line.startsWith(key + "=")) return "";
    return line.substring(key.length() + 1);
}

inline PlayerConfig loadConfig() {
    PlayerConfig c;
    File f = SD.open(CFG_PATH, FILE_READ);
    if (!f) f = SD.open(CFG_PATH_LEGACY, FILE_READ);
    if (!f) return c;
    while (f.available()) {
        String ln = f.readStringUntil('\n'); ln.trim();
        String v;
        if ((v = _cfgVal(ln, "sf2")).length())       c.sf2Path   = v;
        if ((v = _cfgVal(ln, "midi")).length())      c.midiPath  = v;
        if ((v = _cfgVal(ln, "displaymode")).length()) c.displayMode = v.toInt();
        if ((v = _cfgVal(ln, "audiomode")).length()) c.audioMode = v.toInt();
        if ((v = _cfgVal(ln, "mididx")).length())    c.midiIdx   = v.toInt();
    }
    f.close();
    Serial.printf("[CFG] sf2=%s midi=%s dmode=%d mode=%d idx=%d\r\n",
        c.sf2Path.c_str(), c.midiPath.c_str(), c.displayMode, c.audioMode, c.midiIdx);
    return c;
}

inline void saveConfig(const PlayerConfig& c) {
    if (!SD.exists(CFG_DIR) && !SD.mkdir(CFG_DIR)) {
        Serial.printf("[CFG] Failed to create %s\r\n", CFG_DIR);
        return;
    }
    File f = SD.open(CFG_PATH, FILE_WRITE);
    if (!f) { Serial.println("[CFG] Write failed!"); return; }
    f.printf("sf2=%s\n",       c.sf2Path.c_str());
    f.printf("midi=%s\n",      c.midiPath.c_str());
    f.printf("displaymode=%d\n", c.displayMode);
    f.printf("audiomode=%d\n", c.audioMode);
    f.printf("mididx=%d\n",    c.midiIdx);
    f.close();
    Serial.printf("[CFG] Saved %s\r\n", CFG_PATH);
}
