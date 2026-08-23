#!/usr/bin/env python3
"""
Generate flowchart + algorithm PDFs for the Smart Wagon gateway and each
sub-node type.

THE CODE IS THE SOURCE OF TRUTH. Every diagram and table here is written to
match GATEWAY/src and SUBNODE_NODES/*/src as they actually are. Where a block
is specified but NOT YET IMPLEMENTED, it is marked "NOT IMPLEMENTED" rather
than described as working - a document that flatters the build is worse than no
document when it goes to RDSO.

Toolchain: Graphviz `dot` for the diagrams, and for the PDF whichever of
wkhtmltopdf / WeasyPrint / xhtml2pdf is present. Missing tools degrade to HTML
+ .dot files instead of failing the whole run.
"""
import os, subprocess, base64, shutil, sys

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "Flowchart & Algorithms")
os.makedirs(OUT, exist_ok=True)


def find_dot():
    """Graphviz is not on PATH under a default Windows install."""
    p = shutil.which("dot")
    if p:
        return p
    for c in (r"C:\Program Files\Graphviz\bin\dot.exe",
              r"C:\Program Files (x86)\Graphviz\bin\dot.exe"):
        if os.path.isfile(c):
            return c
    return None


DOT = find_dot()

# ---------- graphviz helpers ----------
DOT_HEADER = '''digraph G {
  rankdir=TB; bgcolor="transparent";
  node [shape=box style="rounded,filled" fillcolor="#eef4fb" color="#4c78b5"
        fontname="Helvetica" fontsize=11 margin="0.14,0.09"];
  edge [color="#5b7188" fontname="Helvetica" fontsize=9];
  { node [shape=diamond fillcolor="#fdf0d5" color="#c79a2e"] %s }
'''


def render_png(dot, name):
    dpath = os.path.join(OUT, name + ".dot")
    ppath = os.path.join(OUT, name + ".png")
    with open(dpath, "w", encoding="utf-8") as f:
        f.write(dot)
    if not DOT:
        print(f"  ! graphviz 'dot' not found - kept {name}.dot, no image")
        return None
    subprocess.run([DOT, "-Tpng", "-Gdpi=170", dpath, "-o", ppath], check=True)
    return ppath


def img_tag(path):
    if not path:
        return ('<p class="small"><i>[diagram omitted - Graphviz not installed; '
                'render the matching .dot file]</i></p>')
    b64 = base64.b64encode(open(path, "rb").read()).decode()
    return f'<img src="data:image/png;base64,{b64}"/>'


# ---------- html/pdf ----------
# xhtml2pdf's built-in Helvetica is a Latin-1 Type 1 font: every arrow, >=, x,
# degree and em-dash in this document would render as a black box. Embedding a
# real TrueType face fixes the whole class of problem in one place. Falls back
# to Helvetica (and mangled symbols) only if no system font is found.
def register_font():
    """Register Arial with ReportLab under one family name and tell xhtml2pdf
    about it. CSS @font-face is NOT used - xhtml2pdf copies the face to a temp
    file and the copy fails, so registration has to happen up front."""
    faces = [("DocSans", "arial.ttf", 0, 0), ("DocSans-B", "arialbd.ttf", 1, 0),
             ("DocSans-I", "ariali.ttf", 0, 1), ("DocSans-BI", "arialbi.ttf", 1, 1)]
    root = os.path.join(os.environ.get("WINDIR", r"C:\Windows"), "Fonts")
    if not all(os.path.isfile(os.path.join(root, f[1])) for f in faces):
        return None
    try:
        from reportlab.pdfbase import pdfmetrics
        from reportlab.pdfbase.ttfonts import TTFont
        from reportlab.lib.fonts import addMapping
        from xhtml2pdf import default
        for name, fn, bold, ital in faces:
            pdfmetrics.registerFont(TTFont(name, os.path.join(root, fn)))
            addMapping("DocSans", bold, ital, name)
        default.DEFAULT_FONT["docsans"] = "DocSans"
        return "DocSans"
    except Exception:
        return None


BODY_FONT = register_font() or "Helvetica, Arial, sans-serif"

CSS = '''
@page { size: A4; margin: 16mm 15mm; }
body { font-family: ''' + BODY_FONT + '''; color:#1b2733; font-size:11px; line-height:1.5; }
h1 { font-size:20px; margin:0 0 2px; color:#12385f; }
.sub { color:#5b6b7b; font-size:11px; margin:0 0 14px; }
h2 { font-size:14px; color:#12385f; border-left:4px solid #4c78b5; padding-left:8px; margin:20px 0 6px; }
h3 { font-size:12px; color:#1b4a78; margin:14px 0 4px; }
.flow { text-align:center; margin:10px 0 4px; }
.flow img { max-width:100%; }
table { border-collapse:collapse; width:100%; margin:6px 0 10px; font-size:10.5px; }
th,td { border:1px solid #cdd8e3; padding:4px 7px; text-align:left; vertical-align:top; }
th { background:#eef4fb; color:#12385f; }
code { background:#f0f3f7; padding:1px 4px; border-radius:3px; font-size:10px; }
ol,ul { margin:4px 0 10px 18px; padding:0; }
li { margin:2px 0; }
.small { color:#5b6b7b; font-size:10px; }
.warn { background:#fdecea; border-left:4px solid #c0392b; padding:8px 10px; margin:8px 0; }
.note { background:#fff8e1; border-left:4px solid #c79a2e; padding:8px 10px; margin:8px 0; }
'''


def write_pdf(html, pdf):
    """First renderer that works wins. Returns True if a PDF was produced."""
    if shutil.which("wkhtmltopdf"):
        subprocess.run(["wkhtmltopdf", "--enable-local-file-access", "--quiet",
                        "--print-media-type", html, pdf], check=True)
        return True
    try:
        # Pure Python, so it needs no GTK/Pango - the reliable option on Windows.
        from xhtml2pdf import pisa
        with open(html, encoding="utf-8") as src, open(pdf, "wb") as dst:
            return pisa.CreatePDF(src.read(), dest=dst).err == 0
    except Exception:
        pass
    try:
        from weasyprint import HTML
        HTML(filename=html).write_pdf(pdf)
        return True
    except Exception as e:
        print(f"  ! no HTML->PDF renderer available ({e}) - kept the .html")
        return False


def build_pdf(fname, title, subtitle, flow_png, body_html):
    doc = f'''<!DOCTYPE html><html><head><meta charset="utf-8"><style>{CSS}</style></head>
<body>
<h1>{title}</h1>
<p class="sub">{subtitle}</p>
<div class="flow">{img_tag(flow_png)}</div>
{body_html}
</body></html>'''
    hpath = os.path.join(OUT, fname + ".html")
    with open(hpath, "w", encoding="utf-8") as f:
        f.write(doc)
    ok = write_pdf(hpath, os.path.join(OUT, fname + ".pdf"))
    print(("  + " if ok else "  ~ ") + fname + (".pdf" if ok else ".html only"))


REF = ('<p class="small">Ref: RDSO WD-35-MISC-2024 &middot; SmartWagon Telemetry Protocol Rev.1 '
       '(pv=1) &middot; controller nRF54L15 (QFN52). Generated from the firmware sources by '
       'DOCS/gen_pdfs.py - regenerate after changing the code.</p>')

# Reused across every sub-node sheet: what is actually on the board vs. what
# the firmware does with it.
NOT_IMPL = ('<div class="warn"><b>NOT IMPLEMENTED IN FIRMWARE.</b> This sheet describes the '
            '<i>target</i> design. In the current build this node type has no sensor driver: '
            '<code>sensor.c</code> is a placeholder that returns a fixed 25.0 &deg;C and a zero '
            'secondary value, and its overlay declares no I2C bus, so the BMA400 and STS4x fitted '
            'to the board are not read. Everything else on this sheet - BLE, AES-CCM, the battery '
            'gauge, the config window, the cadence logic - IS implemented and working. '
            'Do not field this node type until the sensor driver lands.</div>')

# =====================================================================
# 1) GATEWAY
# =====================================================================
gw_dot = DOT_HEADER % "evk;lightq;dbq;" + '''
  boot[label="Power on"];
  init[label="power_init(): BMA400 rail ON (never cut),\\nNOR+FRAM rail ON -> flash in DEEP-POWER-DOWN,\\nGNSS + SHT40 + modem rails gated OFF.\\nInit: clocks, FRAM, NOR archive, I2C (SHT40+BMA400),\\nmotion_init(), BMA400 INT wake (P1.16),\\nwdt_init() (WDT31), BLE observer (group filter),\\nGNSS thread, EC200"];
  seed[label="Seed debounce: motion_sample(bma400_is_moving())\\nif active -> start 30 s motion-poll timer"];
  first[label="First cycle: acquire_fix -> link_up ->\\nBOOT event + heartbeat + node-health ->\\nflush backlog -> drain cmds -> link_down"];
  setc0[label="arm_schedule(): cadence from DEBOUNCED state\\n(10 min moving / 12 h stopped)"];
  idle[label="SYSTEM ON IDLE  (CPU asleep; BLE scanner + BMA_INT armed)\\nbackground: BLE observer caches node adverts"];

  evk[label="which wake event?"];

  m_poll[label="EV_MPOLL: motion_sample(bma400_is_moving())\\nno radios - I2C burst on always-on BMA400.\\nif !motion_active(): stop poll timer"];
  m_imp[label="EV_IMPACT (BMA_INT): impact_g = read |a|;\\nmotion_sample(true); (re)start poll timer"];
  lightq[label="sub-threshold only?\\n(no shock, no confirmed flip)"];

  dbq[label="did debounce CONFIRM a\\nstart/stop this poll?"];

  acq[label="REPORT PATH: acquire_fix() GNSS rail ON warm fix\\n(V_BCKP kept) -> link_up() modem rail ON,\\nMQTT connect (persistent session, RE-INIT each wake)"];

  a_sched[label="EV_SCHED (RTC):\\nble_sensors_refresh() active scan -> read node cache (miss=1) ->\\nread SHT40 + BMA400 + power -> speed>15? include vibration ->\\ngwalarm_scan() edge detectors -> publish HEARTBEAT ->\\nnode-health -> SENSOR_FAULT if down -> re-sample motion"];
  a_imp[label="EV_IMPACT (shock >= threshold):\\ngwalarm_impact() -> ALARM IMPACT / DERAIL"];
  a_nal[label="EV_NALARM: publish ALARM\\n(mapped: HOT_BEARING / DOOR_UNAUTH / ...)"];
  a_mot[label="EV_MOTION (CONFIRMED): publish EVENT\\nTRAIN_START / TRAIN_STOP (trip id)"];

  tail1[label="if online: flush backlog (original seq/ts) ->\\ndrain dn/cmd (set_*, ota_start/status, reboot) -> resp\\nif offline: storage_append() to the 15 MB NOR archive\\n(alarms/events are CRITICAL - never evicted by a heartbeat)"];
  tail2[label="link_down(): AT+QPOWD, modem rail OFF;\\nrelease_fix(): GNSS rail OFF; arm_schedule()"];

  boot->init->seed->first->setc0->idle;
  idle->evk[label="wake"];
  evk->m_poll[label="EV_MPOLL"];
  evk->m_imp[label="EV_IMPACT"];
  evk->acq[label="EV_SCHED / EV_NALARM /\\nEV_MOTION"];
  m_poll->dbq; m_imp->lightq;
  lightq->idle[label="yes: back to\\nidle, no radios"];
  lightq->acq[label="no (real shock)"];
  dbq->acq[label="yes -> EV_MOTION"];
  dbq->idle[label="no: back to idle"];
  acq->a_sched[label="EV_SCHED"]; acq->a_imp[label="EV_IMPACT"];
  acq->a_nal[label="EV_NALARM"]; acq->a_mot[label="EV_MOTION"];
  a_sched->tail1; a_imp->tail1; a_nal->tail1; a_mot->tail1;
  tail1->tail2->idle;
}'''
gw_png = render_png(gw_dot, "gateway_flow")

pc_dot = DOT_HEADER % "" + '''
  w[label="Wake (RTC / BMA_INT / BLE alarm)"];
  gon[label="GNSS rail ON (SW_LC)\\nwarm fix (V_BCKP keep-alive kept)"];
  mon[label="Modem rail ON + connect\\n(persistent session)"];
  pub[label="Publish uplink (QoS1)"];
  drn[label="Drain dn/cmd -> resp"];
  moff[label="AT+QPOWD -> modem rail OFF"];
  goff[label="GNSS rail OFF"];
  slp[label="System ON idle\\n(BLE scanner + BMA400 INT stay armed)"];
  w->gon->mon->pub->drn->moff->goff->slp; slp->w[label="next wake"];
}'''
pc_png = render_png(pc_dot, "gateway_power")

# Store-and-forward record layout - the part most often got wrong.
st_dot = DOT_HEADER % "fitq;critq;" + '''
  a[label="storage_append(rec, len, critical)"];
  n[label="n = slots_for(len)\\n= ceil((len + 4) / 1024)"];
  fitq[label="head + n > num_slots?"];
  pad[label="head = 0 (records never wrap)\\nif ring empty: tail = 0 too"];
  critq[label="erasing this sector would destroy\\na CRITICAL record, and the incoming\\nrecord is only a heartbeat?"];
  drop[label="return -ENOSPC\\ncaller logs RECORD LOST\\n(the alarm is kept)"];
  er[label="retire overlapped records,\\nerase the 4 KB sector"];
  wr[label="write [magic 0xA5][len:2][kind:1]\\nthen the payload, spanning n slots"];
  up[label="head += n; count++;\\nmeta_save() to FRAM"];
  a->n->fitq;
  fitq->pad[label="yes"]; fitq->critq[label="no"];
  pad->critq;
  critq->drop[label="yes"]; critq->er[label="no"];
  er->wr->up;
}'''
st_png = render_png(st_dot, "gateway_storage")

gw_body = f'''
<h2>1. Purpose &amp; operating mode</h2>
<p>The gateway (data concentrator) is an <b>uplink-first, Class-A</b> device: it sleeps to meet the
energy budget and wakes only on the RTC schedule, a BMA400 impact interrupt, or a sensor-node
alarm advert. On each wake it fixes position, gathers the latest sensor readings, publishes over the
EC200U 4G modem, drains any queued commands, and returns to sleep. When the link is down, reports are
written to the NOR archive and replayed later with their original <code>seq</code>/<code>ts</code>.</p>

<h2>2. Algorithm</h2>
<h3>2.1 Boot &amp; sync</h3>
<ol>
<li>Init clocks (HFXO/LFXO), FRAM (config, <code>seq</code>, archive pointers), NOR archive, I2C sensors,
BMA400 impact interrupt (P1.16), the WDT31 watchdog, and the low-power BLE observer with the
wagon-group filter.</li>
<li>SYNC: power the GNSS rail, obtain the first fix, set the RTC from UTC, power the rail off (keep-alive stays on for warm starts).</li>
<li><b>Read motion first:</b> from the GNSS speed decide running vs stopped, and set the initial report cadence accordingly (<b>10 min moving / 12 h stopped</b>) before going idle.</li>
<li>Connect: bring the modem up, MQTT CONNECT with a persistent session (<code>clean_session=0</code>), publish the retained <code>status</code> birth, subscribe <code>dn/cmd</code>.</li>
<li>Enter <b>System ON idle</b> (CPU asleep, <b>not</b> System OFF) so the BLE scanner and BMA400 interrupt stay armed and can wake the CPU.</li>
</ol>

<h3>2.2 Scheduled heartbeat (10 min running / 12 h stopped)</h3>
<ol>
<li>Warm GNSS fix (bounded timeout; keep last-known + <code>fix_valid=false</code> on timeout).</li>
<li><b>Collect sensor-node data.</b> The BLE observer caches every node's adverts continuously in the
background. At report time the gateway also runs a short <b>active scan window</b> to refresh any
stale/missing node, then reads the cache for every <b>provisioned</b> node; a node still not heard is
reported <code>"miss":1</code>.</li>
<li>Read SHT40 (temp/humidity), BMA400 motion, rail/health, backup reserve.</li>
<li>Bearing vibration health is only valid at <b>speed &gt; 15 km/h</b> (RDSO &sect;7.9).</li>
<li><b>Node-down detection:</b> any provisioned node not heard within ~15 min is flagged DOWN, a
<code>SENSOR_FAULT</code> alarm is sent (<code>st:set</code>), and a <code>clear</code> when it
reappears. In the heartbeat that node also shows <code>"miss":1</code>.</li>
<li>Run <code>gwalarm_scan()</code> - the edge-triggered detector set in section 3.</li>
<li>Build the <code>hb</code> envelope (seq++, UTC epoch + derived CEP50), publish QoS1 on <code>up/hb</code>.</li>
<li><b>Re-evaluate cadence</b> from the freshly-updated motion state before returning to idle.</li>
</ol>

<h3>2.3 Position quality (RDSO &sect;7.17)</h3>
<p>CEP50 and the constellation list are <b>derived from the receiver</b>, not asserted as constants.
<code>gnss_cep_m()</code> computes CEP50 &asymp; HDOP &times; 2.5 m from the GGA sentence and returns
99 when there is no usable fix, so the cloud can tell a good fix from a bad one.
<code>gnss_sys_json()</code> builds <code>loc.sys</code> from the NMEA <b>talker IDs actually seen</b>,
so it reports provenance rather than intent.</p>
<div class="note"><b>Open point:</b> <code>gnss.c</code> documents that the LC29H does not support NavIC
and never enables it, while the module's own protocol notes say it does. Resolve against the exact
LC29H variant before claiming NavIC in a compliance submission.</div>

<h2>3. Alarms &amp; events generated by the gateway</h2>
<p>All of these are edge-triggered in <code>gwalarm.c</code> against per-node previous state, so a
condition that stays true does not re-alarm every heartbeat.</p>
<table>
<tr><th>Detector</th><th>Emits</th><th>Gate / note</th></tr>
<tr><td>Impact magnitude</td><td>IMPACT / DERAIL</td><td>two-level on <code>|a|</code>; DERAIL above the higher band</td></tr>
<tr><td>Bearing vibration</td><td>FLAT_WHEEL</td><td>only while speed &gt; <code>VIBRATION_MIN_KMH</code> (&sect;7.9)</td></tr>
<tr><td>Load</td><td>OVERLOAD, LOAD_CHANGE</td><td>alarm + state-change event</td></tr>
<tr><td>Battery</td><td>LOW_BATTERY, NODE_LOW_BATTERY</td><td>gateway cell and per-node cell</td></tr>
<tr><td>Door</td><td>DOOR_OPEN / DOOR_CLOSE</td><td>DOOR_UNAUTH is decided with geofence + motion context</td></tr>
<tr><td>Handbrake</td><td>HANDBRAKE_APPLIED / _RELEASED</td><td>HANDBRAKE_MOVING correlates state with GNSS speed</td></tr>
<tr><td>Power path</td><td>CHARGE_START / _STOP, SRC_SWITCH, BAND_CHANGE</td><td>from the BQ25798 charger</td></tr>
<tr><td>Geofence</td><td>GEOFENCE_ENTER / _EXIT</td><td><b>disabled</b>: <code>GEOFENCE_RADIUS_M 0</code>, no provisioning command exists in the protocol</td></tr>
<tr><td>Link</td><td>CONN_RESTORED</td><td>on <code>gwalarm_link_up()</code></td></tr>
</table>

<h3>3.1 Motion debounce (a 1-min start/stop must NOT flip the state)</h3>
<table>
<tr><th>Transition</th><th>Must persist for</th><th>Effect</th></tr>
<tr><td>stopped &rarr; moving</td><td><code>MOTION_START_CONFIRM_MS</code> = 2 min</td><td>confirm &rarr; TRAIN_START, cadence &rarr; 10 min</td></tr>
<tr><td>moving &rarr; stopped</td><td><code>MOTION_STOP_CONFIRM_MS</code> = 5 min</td><td>confirm &rarr; TRAIN_STOP, cadence &rarr; 12 h</td></tr>
<tr><td>blip shorter than the above</td><td>&mdash;</td><td>pending change <b>cancelled</b>: no event, no cadence flip</td></tr>
</table>
<p class="small">The always-on BMA400 is the cheap gatekeeper: while a transition is pending the
gateway re-samples it every <code>MOTION_POLL_MS</code> over I2C with <b>no GNSS and no modem</b>,
purely to time the persistence. Only the confirmed state is ever acted on.</p>

<h2>4. Store-and-forward (RDSO &sect;7.5) - "no data loss at any cost"</h2>
<div class="flow">{img_tag(st_png)}</div>
<p>The archive is a ring of <b>1 KB slots</b> over the 15 MB NOR <code>archive</code> partition
(15360 slots). <b>Records span as many slots as they need</b> - this is what lets a full ~2.4 KB
heartbeat on a fully-fitted wagon be buffered at all. Head and tail are slot indices held in FRAM, so
they survive resets.</p>
<table>
<tr><th>Property</th><th>Behaviour</th></tr>
<tr><td>Record header</td><td><code>[magic 0xA5][len:2][kind:1]</code> - 4 bytes before the payload</td></tr>
<tr><td>Erase unit</td><td>4 KB NOR sector; slots are written individually into an already-erased sector</td></tr>
<tr><td>Critical records</td><td>An alarm/event is <b>never</b> evicted to make room for a heartbeat - the heartbeat is refused with <code>-ENOSPC</code> and the caller logs RECORD LOST</td></tr>
<tr><td>Corrupt header</td><td><code>resync_tail()</code> scans forward to the next intact record, so one damaged record costs one record - not the whole backlog</td></tr>
<tr><td>Wrap</td><td>Records never wrap the end of the partition; head pads to 0, and an empty ring moves tail with it</td></tr>
<tr><td>Capacity</td><td>~106 days at the &sect;7.4 10-minute moving cadence, against the &sect;7.5 one-month requirement</td></tr>
<tr><td>Format change</td><td>The FRAM <code>MAGIC</code> is versioned; a firmware carrying the old record layout resets the ring instead of misparsing it</td></tr>
</table>

<h2>5. Downlink commands &amp; OTA</h2>
<p>After each uplink the gateway drains <code>dn/cmd</code>: <code>set_threshold</code>,
<code>set_interval</code>, <code>set_gnss</code>, <code>get_status</code>,
<code>ota_start</code>/<code>ota_status</code>, <code>reboot</code> - each answered with a correlated
<code>resp</code>.</p>
<table>
<tr><th>Path</th><th>Transport</th><th>Scope</th></tr>
<tr><td>Server &rarr; gateway</td><td>MQTT</td><td>thresholds, intervals, <b>gateway firmware (MCUboot dual-slot, auto-revert)</b></td></tr>
<tr><td>Gateway &rarr; sub-node</td><td>BLE GATT, during the node's config window</td><td><b>thresholds only</b> - one authenticated 20-byte write</td></tr>
</table>
<div class="note"><b>Sub-node firmware update does not exist</b> by design. There is no SMP/MCUmgr path
on any node, and no phone can update one. A node's thresholds change only via its own gateway, and the
write is accepted only if it decrypts under the per-wagon key, is addressed to that node, and carries a
strictly higher downlink counter.</div>

<h2>6. Watchdog</h2>
<p>The gateway board has <b>no external watchdog</b> (the sub-nodes have a TPL5010), so the on-chip
WDT31 is the only hang recovery. <code>watchdog.c</code> uses a <b>liveness gate</b>: the main loop
must mark itself alive for the feed to happen, and long legitimate idles are bracketed with
<code>gw_wdt_idle_begin/end</code> so sleeping is not mistaken for hanging. Timeout 120 s, feed 30 s,
10 min idle margin. Expiry causes a software reset.</p>

<h2>7. Key parameters</h2>
<table>
<tr><th>Item</th><th>Value</th></tr>
<tr><td>Heartbeat cadence</td><td>10 min moving / 12 h stopped</td></tr>
<tr><td>Motion debounce</td><td>start confirm 2 min / stop confirm 5 min; poll every 30 s</td></tr>
<tr><td>Impact alarm threshold</td><td>|a| &ge; 4.0 g <i>(vendor-set; &sect;7.15 gives a 6-month calibration window)</i></td></tr>
<tr><td>Bearing-vibration valid speed</td><td>&gt; 15 km/h</td></tr>
<tr><td>Critical-alarm RX window</td><td><b>listen 100 ms every 5 s</b> (<code>RX_WINDOW_MS</code> / <code>RX_INTERVAL_MS</code>)</td></tr>
<tr><td>Offline buffer</td><td>15 MB NOR ring, ~106 days (&sect;7.5 needs 1 month)</td></tr>
<tr><td>Position accuracy</td><td>CEP50 derived from HDOP; 99 reported when no fix</td></tr>
</table>

<h2>8. Power sequencing (per wake)</h2>
<div class="flow">{img_tag(pc_png)}</div>
<table>
<tr><th>Rail (switch)</th><th>State</th><th>Powered when</th></tr>
<tr><td>BMA400 (SW_AXI, P2.00)</td><td>always ON</td><td>impact/tamper wake source; never cut</td></tr>
<tr><td>GNSS main (SW_LC P0.01 + GNSS_PWR_EN P1.09)</td><td>gated</td><td>only around a fix; V_BCKP keep-alive stays on</td></tr>
<tr><td>EC200U modem (SW_GNSS, P2.08)</td><td>gated</td><td>only to publish; AT+QPOWD then off</td></tr>
<tr><td>SHT40 (SW_SHT_LC, P0.05)</td><td>gated</td><td>only during a climate read</td></tr>
<tr><td>NOR+FRAM (SW_MEM, P0.04)</td><td><b>always ON</b></td><td>rail kept up; flash held in deep-power-down instead</td></tr>
</table>

<h3>8.1 What needs re-configuring on wake</h3>
<table>
<tr><th>Device</th><th>Rail between reports</th><th>Loses config?</th><th>On wake</th></tr>
<tr><td>BMA400</td><td>always ON</td><td>No</td><td><b>Nothing</b> - range/ODR/INT map retained</td></tr>
<tr><td>W25Q128 NOR + FRAM</td><td>always ON (DPD)</td><td>No</td><td>Release deep-power-down only</td></tr>
<tr><td>LC29H GNSS</td><td>gated (V_BCKP kept)</td><td>No (warm)</td><td>Release RESET_N; ephemeris preserved</td></tr>
<tr><td>SHT40</td><td>gated</td><td>No (stateless)</td><td>Nothing - each read is a fresh single-shot</td></tr>
<tr><td>EC200U modem</td><td>gated (off)</td><td>Yes (full)</td><td>Full re-init: CFUN/CPIN &rarr; CGATT &rarr; QIACT &rarr; QMTOPEN/QMTCONN</td></tr>
</table>

<h2>9. Hardware verification status</h2>
<div class="warn"><b>Nothing in this project has run on hardware yet.</b> The pin map below was
verified against the gateway schematic netlist; the items marked open are firmware gaps or bench
checks that must be closed before production flashing.</div>
<table>
<tr><th>Item</th><th>Status</th></tr>
<tr><td>All assigned nRF54L15 pins vs. schematic netlist</td><td><b>Verified</b> - UARTs, I2C, SPI memory bus, 4 power switches, charger, GNSS control, BMA_INT</td></tr>
<tr><td>I2C addresses: BMA400 <code>0x14</code>, SHT40 <code>0x44</code>, BQ25798 <code>0x6B</code></td><td><b>Verified</b> against the schematic</td></tr>
<tr><td><code>GNSS_3V3</code> (P2.09) &rarr; TXS0102 pin 7 (VCCB)</td><td><b>OPEN - firmware never drives it.</b> This GPIO is the supply for the level shifter on the EC200U UART; while it is low, that UART cannot work in either direction. Prime suspect for the silent modem.</td></tr>
<tr><td><code>SW_PWR</code> (P2.10)</td><td>OPEN - no firmware use; destination not resolvable from the drawing</td></tr>
<tr><td><code>WP</code> / <code>FRAM_WP</code> (P2.06 / P2.07)</td><td>Not driven - safe, both have 10 k&Omega; pull-ups so writes stay enabled</td></tr>
<tr><td>PWRKEY / RESET polarity through T1/T2</td><td>OPEN - GPIO high pulls the modem line low; verify on the bench</td></tr>
</table>
{REF}
'''
build_pdf("Gateway_Algorithm", "Smart Wagon Gateway — Flowchart &amp; Algorithm",
          "Data concentrator (Class A) &middot; nRF54L15 + LC29HBA GNSS + EC200U 4G + BMA400/SHT40",
          gw_png, gw_body)

# =====================================================================
# 1b) SYSTEM OVERVIEW & SENSOR ROLES
# =====================================================================
sys_dot = '''digraph S {
  rankdir=LR; bgcolor="transparent"; compound=true; nodesep=0.35; ranksep=0.7;
  node [shape=box style="rounded,filled" fontname="Helvetica" fontsize=11 margin="0.16,0.10"];
  edge [color="#5b7188" fontname="Helvetica" fontsize=9];

  subgraph cluster_nodes {
    label="Sensor sub-nodes (19 per wagon)"; fontname="Helvetica"; fontsize=10;
    color="#c79a2e"; style="rounded"; bgcolor="#fdf7e8";
    bn[label="Bearing x8\\n(STUB)" fillcolor="#fdecea" color="#c0392b"];
    lt[label="Load / tilt x4\\n(STUB)" fillcolor="#fdecea" color="#c0392b"];
    hb[label="Handbrake x1\\n(STUB)" fillcolor="#fdecea" color="#c0392b"];
    dr[label="Door / hatch x4\\n(implemented)" fillcolor="#e7f0e7" color="#4a8a4a"];
    tk[label="Tank temp x2\\n(implemented)" fillcolor="#e7f0e7" color="#4a8a4a"];
  }

  subgraph cluster_gw {
    label="GATEWAY  (nRF54L15, QFN52)"; fontname="Helvetica"; fontsize=10;
    color="#4c78b5"; style="rounded"; bgcolor="#eef4fb";
    mcu[label="nRF54L15 MCU\\nZephyr RTOS\\nSystem ON idle\\nstore-and-forward" fillcolor="#dbe8f7" color="#2f5c8f"];
    bma[label="BMA400 accel\\n(always ON)" fillcolor="#e7f0e7" color="#4a8a4a"];
    sht[label="SHT40\\ntemp/humidity" fillcolor="#e7f0e7" color="#4a8a4a"];
    gns[label="LC29H GNSS" fillcolor="#e7f0e7" color="#4a8a4a"];
    mem[label="NOR 15 MB + FRAM\\n(archive+config)" fillcolor="#e7f0e7" color="#4a8a4a"];
    mdm[label="EC200U 4G\\nmodem" fillcolor="#e7f0e7" color="#4a8a4a"];
  }

  cloud[label="MQTT broker /\\ncloud back-office" shape=box style="rounded,filled" fillcolor="#f0e7f5" color="#8a5ca8"];

  bn->mcu [ltail=cluster_nodes lhead=cluster_gw label="BLE adverts\\n(wgn_group filter)"];
  bma->mcu [dir=both label="I2C + INT (P1.16)\\nmotion / impact wake"];
  sht->mcu [label="I2C"];
  gns->mcu [dir=both label="UART\\nposition / speed"];
  mem->mcu [dir=both label="SPI"];
  mcu->mdm [label="UART / AT"];
  mdm->cloud [dir=both label="MQTT/TLS over 4G"];
}'''
sys_png = render_png(sys_dot, "system_overview")

sys_body = f'''
<h2>1. What the system is</h2>
<p>Each freight wagon carries one <b>gateway</b> (the nRF54L15 board) and <b>19 sensor sub-nodes</b>.
The sub-nodes are battery BLE broadcasters that each watch one thing and shout a short encrypted
advert. The gateway listens for its own wagon's nodes, adds its <b>own</b> measurements (position,
motion, climate), and forwards everything to the cloud over 4G - buffering to flash whenever the
network is down.</p>

<h2>2. Implementation status by node type</h2>
<div class="warn">Every sub-node board carries a <b>BMA400 accelerometer</b> and an <b>STS4x internal
temperature sensor</b>. Only the DOOR and TANK firmware actually reads them. The other 13 nodes build,
encrypt and transmit correctly but report a <b>fabricated constant</b>.</div>
<table>
<tr><th>Node type</th><th>Qty</th><th>Sensor driver</th><th>What it reports today</th></tr>
<tr><td><b>DOOR</b></td><td>4</td><td>BMA400 + STS4x + reed, implemented</td><td>Real internal temp, real peak |a|, real door state</td></tr>
<tr><td><b>TANK_TEMP</b></td><td>2</td><td>RTD + BMA400 + STS4x, implemented</td><td>Real cargo temp (needs calibration) + real internal temp</td></tr>
<tr><td><b>BEARING</b></td><td>8</td><td><b>none - stub</b></td><td>Fixed 25.0 &deg;C, vibration 0</td></tr>
<tr><td><b>LOAD_TILT</b></td><td>4</td><td><b>none - stub</b></td><td>Fixed 25.0 &deg;C, secondary 0</td></tr>
<tr><td><b>HANDBRAKE</b></td><td>1</td><td><b>none - stub</b></td><td>Fixed 25.0 &deg;C, secondary 0</td></tr>
</table>
<p class="small">The stub nodes also declare no I2C bus in their devicetree overlay, so the fitted
BMA400 and STS4x are not merely unread - they are unreachable until the overlay gains the bus, the
rail switch and the interrupt line that the DOOR overlay already has.</p>

<h2>3. Why there are two "movement" sensors</h2>
<table>
<tr><th></th><th>BMA400 accelerometer</th><th>LC29H GNSS</th></tr>
<tr><td><b>Question it answers</b></td><td>"Is the wagon vibrating / has it been shocked <b>right now</b>?"</td><td>"<b>Where</b> is the wagon and <b>how fast</b> is it going?"</td></tr>
<tr><td><b>Power</b></td><td>microamps - can stay on 24/7</td><td>tens of milliamps - short bursts only</td></tr>
<tr><td><b>Always on?</b></td><td><b>Yes</b> - it is the wake-up trigger</td><td><b>No</b> - powered only around a fix</td></tr>
<tr><td><b>Role</b></td><td><b>Gatekeeper</b>: wakes the CPU, feeds the debounce</td><td><b>Measurer</b>: exact position + speed for the report</td></tr>
</table>
<p>The accelerometer notices movement for microamps and wakes the system; only then does the expensive
GNSS turn on. Using GNSS to continuously watch for movement would flatten the battery in weeks.</p>

<h2>4. Role of every block</h2>
<table>
<tr><th>Block</th><th>Its job</th></tr>
<tr><td><b>BMA400</b></td><td>Always-on gatekeeper: vibration, high-g impact/derailment, and the CPU wake interrupt (P1.16). On DOOR nodes it also provides the &sect;7.3 always-on tamper watch.</td></tr>
<tr><td><b>LC29H GNSS</b></td><td>Position and speed. Speed gates bearing-vibration validity (&gt; 15 km/h) and door/handbrake context.</td></tr>
<tr><td><b>SHT40</b> (gateway) / <b>STS4x</b> (nodes)</td><td>Local climate / node internal temperature. Stateless single-shot reads.</td></tr>
<tr><td><b>NOR + FRAM</b></td><td>Store-and-forward: ~106 days of reports when 4G is down, replayed with original seq/ts. FRAM holds config and ring pointers.</td></tr>
<tr><td><b>EC200U 4G</b></td><td>The uplink. Fully powered down between reports.</td></tr>
<tr><td><b>BLE observer</b></td><td>Caches this wagon's node adverts (filtered by <code>wgn_group</code>) and catches alarm adverts in a 100 ms / 5 s receive window.</td></tr>
</table>

<h2>5. One report, end to end</h2>
<ol>
<li><b>Asleep</b> (System ON idle): only the BMA400 and the BLE observer are awake.</li>
<li><b>Wake</b>: schedule timer, BMA400 interrupt, or a node alarm.</li>
<li><b>Measure</b>: GNSS warm fix; SHT40 + BMA400; each provisioned node's cached reading (down &rarr; SENSOR_FAULT).</li>
<li><b>Send</b>: modem on, MQTT connect, publish, drain downlink commands.</li>
<li><b>Sleep</b>: GNSS and modem off, re-check debounced motion to set the next cadence, back to idle.</li>
</ol>
{REF}
'''
build_pdf("System_Overview", "Smart Wagon — System Overview &amp; Sensor Roles",
          "How the gateway, sub-nodes and each sensor fit together &middot; nRF54L15 (QFN52)",
          sys_png, sys_body)


# =====================================================================
# 2) SUB-NODE template
# =====================================================================
def subnode(fname, title, subtitle, meas_label, eval_label, alarm_label,
            payload_rows, alarm_rows, extra_html="", implemented=True):
    dot = DOT_HEADER % "evalq;cfgq;" + f'''
  boot[label="Power on"];
  init[label="Init sensor; load thresholds from NVS;\\nderive wgn_group + per-wagon key from WAGON_NUMBER;\\nbatt_init() - coulomb gauge, detects a new cell"];
  sleep[label="SLEEP (this node's OWN RTC timer)\\nnormal ~30 s   /   alarm ~1 s\\nbatt_account_idle()"];
  meas[label="{meas_label}\\nbatt_account_meas()"];
  evalq[label="{eval_label}"];
  norm[label="flags.ALARM=0\\nbroadcast at NORMAL cadence (~30 s)"];
  alrm[label="flags.ALARM=1\\nbroadcast at FAST cadence (~1 s, hold)"];
  pack[label="Build reading (flags,batt,value,value2)\\nSEAL: AES-CCM encrypt + 4B MIC\\n(ctr nonce, per-wagon key)"];
  tx[label="BLE non-connectable advertise\\n(encrypted sw_adv_enc, 21 B)\\nbatt_account_tx()"];
  cfgq[label="Config window due?\\n(every 20 cycles, not in alarm)"];
  cfg[label="Open CONNECTABLE config window (4 s)\\ngateway may write ONE authenticated\\n20-byte threshold frame\\nbatt_account_cfgwin()"];

  cad[shape=note fillcolor="#fff8e1" color="#c79a2e"
      label="CADENCE = this node's OWN timer (~30 s / ~1 s).\\nThe gateway's 10 min / 12 h is SEPARATE -\\nthat is the GATEWAY's MQTT report clock."];
  rel[shape=note fillcolor="#eef4fb" color="#4c78b5"
      label="This node BROADCASTS - it is never polled.\\nThe gateway scans 100 ms every 5 s (2%),\\ncaches the latest advert, and forwards the cache\\non its own 10 min / 12 h wake. No request/response.\\nThe ~1 s alarm rate guarantees capture within ~5 s."];
  sec[shape=note fillcolor="#fdecea" color="#c0392b"
      label="NO FIRMWARE UPDATE PATH.\\nNo SMP, no MCUmgr, no phone update.\\nThe window accepts thresholds ONLY, and only if the\\nframe decrypts under the per-wagon key, is addressed\\nto THIS node, and beats the stored downlink counter."];

  boot->init->sleep->meas->evalq;
  evalq->alrm[label="breach"]; evalq->norm[label="normal\\n(hysteresis)"];
  norm->pack; alrm->pack; pack->tx->cfgq;
  cfgq->cfg[label="yes"]; cfgq->sleep[label="no"];
  cfg->sleep;
  cad->sleep[style=dashed dir=none color="#c79a2e"];
  rel->tx[style=dashed dir=none color="#4c78b5"];
  sec->cfg[style=dashed dir=none color="#c0392b"];
}}'''
    png = render_png(dot, fname + "_flow")
    pr = "".join(f"<tr><td>{a}</td><td>{b}</td></tr>" for a, b in payload_rows)
    ar = "".join(f"<tr><td>{a}</td><td>{b}</td><td>{c}</td></tr>" for a, b, c in alarm_rows)
    body = f'''
{"" if implemented else NOT_IMPL}
<h2>1. Role</h2>
<p>A battery-powered, <b>connectionless BLE broadcaster</b>. It measures its sensor, <b>encrypts</b> the
reading into a BLE advertisement (AES-CCM, with the wagon-group id for isolation), broadcasts a short
burst, and sleeps. On a threshold breach it switches to fast "alarm" advertising so the gateway's
receive window catches it within ~5 s. It forms a connection only during a periodic <b>config
window</b>, and only to receive thresholds.</p>

<div class="note"><b>Timing &mdash; no polling:</b> this node advertises on its <b>own</b> timer &mdash;
about every <b>30&nbsp;s</b> normally, <b>~1&nbsp;s</b> while in alarm. The gateway's
<b>10&nbsp;min / 12&nbsp;h</b> cadence is a <b>separate</b> clock. This node is <b>never polled</b>: it
broadcasts blind, the gateway's low-power scan (<b>100&nbsp;ms every 5&nbsp;s, 2% duty</b>) caches the
latest advert, and on the gateway's own schedule it forwards that cache upward.</div>

<h2>2. Algorithm</h2>
<ol>
<li>Init: load <code>node_id</code>, <code>wgn_group</code> and thresholds from NVS; start the coulomb
gauge (which auto-zeroes if it detects a replacement cell).</li>
<li>Wake on the node's own RTC timer; {meas_label.replace(chr(92) + 'n', ' ').lower()}.</li>
<li>Evaluate against the alarm threshold <b>with hysteresis</b> ({alarm_label}).</li>
<li><b>Seal</b> the reading with AES-CCM under the per-wagon key, using a fresh monotonic counter
<code>ctr</code> as the nonce (persisted in NVS so it never repeats). Nonce byte 7 carries a direction
tag so an uplink nonce can never collide with a downlink one.</li>
<li>Advertise the encrypted burst (non-connectable); <b>slow when normal (~30 s), fast while in alarm (~1 s)</b>.</li>
<li>Every 20 normal cycles (never during an alarm) open a 4 s connectable <b>config window</b> so the
gateway can write new thresholds. <b>No firmware can be loaded through it.</b></li>
</ol>

<h2>3. Advertisement payload (encrypted <code>sw_adv_enc</code>, 21 B)</h2>
<p class="small">Cleartext header (company, ver, wgn_group, type, id, ctr) + <b>AES-CCM ciphertext</b> of the
fields below + 4-byte MIC. The header is authenticated; the reading itself is encrypted.</p>
<table><tr><th>Field</th><th>Meaning for this node</th></tr>{pr}</table>

<h2>4. Alarm / event mapping</h2>
<table><tr><th>Condition</th><th>Protocol code</th><th>Notes</th></tr>{ar}</table>

<h2>5. Battery gauge (coulomb counting)</h2>
<p>The cell is a <b>non-rechargeable 13 Ah Li-SOCl&sub2; ER34615</b>. Its discharge curve is flat, so a
voltage-to-percent map reads ~100 % for years and then collapses. State of charge is therefore obtained
by <b>integrating what the firmware spends</b>: every sleep, measurement, advertising burst and config
window adds (current &times; time) in nAh to an accumulator persisted in NVS. Self-discharge is carried
as a constant current so elapsed time is never free.</p>
<table>
<tr><th>Item</th><th>Value</th></tr>
<tr><td>Usable capacity</td><td><code>BATT_USABLE_MAH</code> = 10000 (derated from the 13 Ah nameplate for cold and load)</td></tr>
<tr><td>Total current ceiling</td><td>13 Ah over a 6-year life = <b>247 &micro;A average</b></td></tr>
<tr><td>Predicted average</td><td>~134 &micro;A (sleep 5 + self-discharge 15 + measure 1 + advertising 60 + config window 53)</td></tr>
<tr><td>Independent end-of-life check</td><td><code>BATT_EOL_MV</code> = 3000 mV, so counter drift cannot hide a dying cell</td></tr>
</table>
<div class="warn"><b>Calibration outstanding:</b> every <code>BATT_I_*_UA</code> constant except
self-discharge is an estimate marked <code>MEASURE:</code> in the source. They must be measured with a
Nordic PPK2 in series with the cell over one full wake cycle before the projected life means anything.</div>
{extra_html}
{REF}
'''
    build_pdf(fname, title, subtitle, png, body)


common_payload = [
    ("wgn_group", "This wagon's isolation group, from the wagon number (gateway drops other groups; cleartext)"),
    ("node_type / node_id", "sensor type + unique id within the wagon (cleartext header)"),
    ("ctr", "monotonic nonce counter, persisted in NVS (cleartext; makes each advert unique)"),
    ("flags / batt (encrypted)", "ALARM/LOWBATT flags, battery % from the coulomb gauge"),
]

# ---- BEARING (stub) ----
subnode("Subnode_Bearing", "Sub-node — Bearing &amp; Wheel Health",
        "8 per wagon &middot; bearing temperature + vibration signature",
        "Measure bearing temp\\n+ vibration (FFT)",
        "temp &gt; limit OR\\nflat-wheel signature?",
        "on temp &ge; 85&deg;C, off &lt; 80&deg;C",
        common_payload + [("value", "bearing temperature &times;10 (&deg;C)"),
                          ("value2", "vibration index (valid &gt; 15 km/h)")],
        [("Bearing over-temp / seizure", "HOT_BEARING", "sev red; drives escalation"),
         ("Wheel flat / RCF (vibration)", "FLAT_WHEEL", "only while speed &gt; 15 km/h (&sect;7.9)"),
         ("Cell low", "NODE_LOW_BATTERY", "advisory, rides next heartbeat")],
        extra_html='''<h2>6. What is missing</h2>
<ul>
<li>Bearing temperature front end - no schematic supplied for this board, so neither the sensor type
(NTC / RTD / I2C part) nor its pin is known to the firmware.</li>
<li>Vibration acquisition: burst-sample the BMA400 and reduce to an index (RMS or FFT band energy).
The algorithm and its calibration are not defined anywhere in the current documents.</li>
<li>Devicetree: no I2C bus, no sensor rail switch, no BMA400 interrupt line in this node's overlay.</li>
</ul>''',
        implemented=False)

# ---- LOAD / TILT (stub) ----
subnode("Subnode_LoadTilt", "Sub-node — Load &amp; Tilt Detection",
        "4 per wagon &middot; load status + tilt angle",
        "Measure load (%)\\n+ tilt angle",
        "load &gt; limit OR\\ntilt &gt; safe angle?",
        "on angle &ge; set, off &lt; set&minus;hyst",
        common_payload + [("value", "load (%)"), ("value2", "tilt &times;10 (deg)")],
        [("Load over configured limit", "OVERLOAD", "sev warn/crit"),
         ("Tilted / asymmetric load", "TILT", "safety"),
         ("Loaded / unloaded at yard", "LOAD_CHANGE (event)", "state change")],
        extra_html='''<h2>6. What is missing</h2>
<ul>
<li>Tilt can be derived directly from the fitted BMA400 (gravity vector &rarr; angle), but no such code
exists on this node.</li>
<li>Load sensing has no defined transducer on this board - no schematic supplied.</li>
<li>Devicetree: no I2C bus, no sensor rail switch, no BMA400 interrupt line.</li>
</ul>''',
        implemented=False)

# ---- HANDBRAKE (stub) ----
subnode("Subnode_Handbrake", "Sub-node — Handbrake Monitoring",
        "1 per wagon &middot; applied / released state",
        "Read handbrake state\\n(engaged / released)",
        "state changed?",
        "engaged=1 / released=0 (debounced)",
        common_payload + [("value", "1 = engaged, 0 = released"), ("value2", "unused")],
        [("Handbrake engaged while moving", "HANDBRAKE_MOVING", "gateway raises it by correlating state + speed"),
         ("Applied / released (stationary)", "HANDBRAKE_APPLIED / _RELEASED (event)", "normal operation")],
        extra_html='''<p class="small">The node only reports state; the gateway decides HANDBRAKE_MOVING
by combining this with GNSS speed.</p>
<h2>6. What is missing</h2>
<ul>
<li>No switch/sensor input is defined in this node's overlay - the handbrake state has no source.</li>
<li>Devicetree: no I2C bus, no sensor rail switch, no BMA400 interrupt line.</li>
</ul>''',
        implemented=False)

# ---- DOOR (implemented) ----
subnode("Subnode_Door", "Sub-node — Door / Hatch Monitoring",
        "4 per wagon &middot; door state + tamper/impact + internal temperature",
        "Read reed aggregate (SW_1),\\nSTS4x internal temp,\\nBMA400 peak |a|",
        "door open, or\\n|a| over tamper threshold?",
        "open=1 / closed=0 (debounced); impact by |a| threshold",
        common_payload + [("value", "STS4x internal temperature &times;10 (&deg;C)"),
                          ("value2", "peak |acceleration| &times;100 (g) - tamper magnitude"),
                          ("flags", "DOOR_OPEN | IMPACT | ALARM | LOWBATT")],
        [("Open in unauthorised zone / moving", "DOOR_UNAUTH", "gateway decides via geofence + motion"),
         ("Authorised in-zone change", "DOOR_OPEN / DOOR_CLOSE (event)", "normal"),
         ("Tamper impact", "DOOR_TAMPER", "<b>off-spec code</b> - not in Protocol Rev.1 &sect;3.2"),
         ("Cell low", "NODE_LOW_BATTERY", "advisory")],
        extra_html='''<h2>6. Hardware &amp; pin map (verified against the DOOR schematic)</h2>
<table>
<tr><th>Function</th><th>Net</th><th>Pin</th></tr>
<tr><td>I2C SDA / SCL (BMA400 + STS4x)</td><td>SDA / SCL</td><td>P1.04 / P1.05 (i2c21)</td></tr>
<tr><td>Sensor-rail enable (TPS22917)</td><td>SW_AXI</td><td>P1.06</td></tr>
<tr><td>BMA400 interrupt INT1</td><td>BMA_INT</td><td>P0.04</td></tr>
<tr><td>Door reed aggregate RS1..RS4</td><td>SW_1</td><td>P1.11 (pull-up)</td></tr>
<tr><td>External watchdog WAKE / DONE (TPL5010)</td><td>uC_Wakeup / uC_Alive</td><td>P0.00 / P0.01</td></tr>
<tr><td>Debug UART</td><td>D_TX / D_RX</td><td>P1.02 / P1.03 (NFC pins released via <code>&amp;uicr</code>)</td></tr>
</table>
<p class="small">The four reeds are wired in <b>parallel on one net</b>, so the firmware sees only the
aggregate: the line reads "open" only when <b>all</b> reeds are open. Per-door sensing needs them split
onto separate GPIOs.</p>

<h2>7. Always-on tamper (RDSO &sect;7.3)</h2>
<p>&sect;7.3 requires sensors to be "always ON to ensure capturing of any and all events". Polling once
per 30 s cycle cannot do that - a tamper between samples is simply lost. On this node the sensor rail
therefore <b>stays up</b> and the BMA400's own motion interrupt wakes the MCU, so an impact is timed by
hardware rather than by the sample clock.</p>
<div class="note"><b>Applied to <code>DOOR_B1D_L</code> only.</b> The interrupt-driven path
(<code>impactint.c</code>) has not yet been propagated to the other three DOOR nodes, which still use a
blind sleep between samples.</div>

<h2>8. Open items</h2>
<ul>
<li><b>STS4x address:</b> the firmware uses <code>0x44</code>; the generated schematic pin-map sheet says
<code>0x4A</code>. Confirm the exact STS4x variant - one of the two is wrong.</li>
<li><b>BMA400 INT1</b> configuration was corrected against the datasheet (activity + OR, non-latched,
push-pull, active high) but has never been confirmed on hardware: verify it fires on a knock and
<i>not</i> on stillness.</li>
<li><code>DOOR_TAMPER</code> is not a Protocol Rev.1 alarm code.</li>
</ul>''')

# ---- TANK (implemented) ----
subnode("Subnode_TankTemp", "Sub-node — Tank / Cargo Temperature",
        "2 per wagon &middot; cargo temperature (&minus;50&hellip;+150&deg;C)",
        "Power sensor rail, read RTD via SAADC,\\nSTS4x internal temp, BMA400 |a|,\\nrail back off",
        "temp outside band?",
        "on &gt; high or &lt; low, off inside band",
        common_payload + [("value", "RTD cargo temperature &times;10 (&deg;C) [primary]"),
                          ("value2", "STS4x internal temperature &times;10 (&deg;C)"),
                          ("flags", "ALARM (over-temp) | IMPACT | LOWBATT")],
        [("Tank/cargo temperature breach", "TANK_OVERTEMP", "immediate alert on breach"),
         ("Tamper impact", "TANK_TAMPER", "<b>off-spec code</b> - not in Protocol Rev.1 &sect;3.2"),
         ("Cell low", "NODE_LOW_BATTERY", "advisory")],
        extra_html='''<h2>6. Hardware &amp; pin map (verified against the TANK schematic)</h2>
<table>
<tr><th>Function</th><th>Net</th><th>Pin</th></tr>
<tr><td>I2C SDA / SCL (BMA400 + STS4x)</td><td>SDA / SCL</td><td>P1.04 / P1.05 (i2c21)</td></tr>
<tr><td>Sensor-rail enable (TPS22917)</td><td>SW_AXI</td><td>P1.06</td></tr>
<tr><td>BMA400 interrupt INT1</td><td>BMA_INT</td><td>P0.04</td></tr>
<tr><td>RTD analog in (LMV321 bridge amp)</td><td>Temp_Sense</td><td>P1.07 / AIN3</td></tr>
<tr><td>External watchdog WAKE / DONE (TPL5010)</td><td>uC_Wakeup / uC_Alive</td><td>P0.00 / P0.01</td></tr>
</table>

<h2>7. Why this node does NOT keep its sensor rail on</h2>
<p>Unlike the DOOR node, the sensor rail here is <b>gated off between reads</b>. The RTD bridge is
excited by a TL431 reference that draws roughly 1 mA continuously, and it shares
<code>VCC_AXI</code> with the BMA400. Leaving that rail up to satisfy &sect;7.3 would cost about four
times the entire 247 &micro;A budget. The trade is deliberate: continuous tamper watch is provided on
the DOOR nodes, and this node powers its rail only to measure.</p>

<h2>8. Open items</h2>
<ul>
<li><b>RTD calibration outstanding.</b> The element type (PT100 vs PT1000) and the LMV321 gain are not
determinable from the drawing, so <code>RTD_CAL_M</code> / <code>RTD_CAL_B</code> must be set from a
two-point bench calibration. <b>Until then the reported cargo temperature is not trustworthy.</b></li>
<li><b>STS4x address</b> <code>0x44</code> in firmware vs <code>0x4A</code> on the pin-map sheet.</li>
<li><code>TANK_TAMPER</code> is not a Protocol Rev.1 alarm code.</li>
</ul>''')

print("done -> " + OUT)
