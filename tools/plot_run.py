#!/usr/bin/env python3
"""Plot a run: the path the car drove (top view, coloured by speed) plus speed,
power and steering against time, as one self-contained HTML page.

Usage:
    python3 tools/plot_run.py run_logs/<time>.csv [--open]
        [--D 5 --B 1 --bottle 2] # only if the log has no PLAN line

Writes <time>.html next to the CSV. Reads the matching <time>.log for the plan
(D, B, bottle) and the car geometry; falls back to src/config.h for the geometry.
capture_run.py calls this for every run it saves.

Everything is what the car THOUGHT: its own telemetry (encoder + IMU heading), in
its own frame - x along and y across the direction it pointed at START (heading 0).
An aim error at START does not show in it. Sideways is in cm and stretched.
Standard library only.
"""
import argparse
import csv
import json
import math
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG_H = os.path.join(HERE, "..", "src", "config.h")


def read_samples(csv_path):
    with open(csv_path) as f:
        rows = list(csv.DictReader(f))
    out = []
    for r in rows:
        try:
            out.append({
                "t": int(r["t_ms"]), "st": r["state"], "x": float(r["x_mp_m"]),
                "v": float(r["v_mps"]), "vref": float(r["vref_mps"]), "pwm": float(r["pwm_pct"]),
                "rx": float(r["roller_x_m"]), "ry": float(r["y_m"]),
                "th": float(r["heading_deg"]), "steer": float(r["steer_deg"]),
                "ok": int(r["steer_ok"]), "stuck": int(r["stuck"]), "launch": int(r["launch"]),
            })
        except (KeyError, ValueError):
            continue                       # a garbled line from the serial port
    return out


def config_value(name):
    try:
        text = open(CONFIG_H).read()
    except OSError:
        return None
    m = re.search(r"constexpr\s+float\s+%s\s*=\s*([-\d.eE+]+)f?" % name, text)
    return float(m.group(1)) if m else None


def read_log(log_path):
    info = {"plan": {}, "geom": {}, "result": {}}
    if not os.path.exists(log_path):
        return info
    text = open(log_path, errors="replace").read()
    m = re.search(r"PLAN D=([\d.]+) B=([\d.]+) T=([\d.]+) bottle@([\d.]+)\s+mpOfs=([-+\d.]+)"
                  r" btlOfs=([-+\d.]+)(?: btlPast=([\d.]+)cm)?(?: aim=([-+\d.]+)deg)?", text)
    if m:
        info["plan"].update(D=float(m[1]), B=float(m[2]), T=float(m[3]), xBottle=float(m[4]),
                            mpOfs=float(m[5]), btlOfs=float(m[6]))
        if m[7]:
            info["plan"]["btlPast"] = float(m[7]) / 100.0
        if m[8]:
            info["plan"]["aim"] = float(m[8])
    m = re.search(r"xF=([\d.]+)\s+xFinal=([\d.]+)", text)
    if m:
        info["plan"].update(xF=float(m[1]), xFinal=float(m[2]))
    m = re.search(r"ENC_FROM_MP_M=([\d.]+) L_P_M=([\d.]+)", text)
    if m:
        info["geom"].update(enc=float(m[1]), lp=float(m[2]))
    m = re.search(r"forward stop: planned ([\d.]+)\s+reached ([\d.]+)", text)
    if m:
        info["result"].update(xFplan=float(m[1]), xFgot=float(m[2]))
        info["plan"].setdefault("xF", float(m[1]))
    m = re.search(r"final MP:\s+planned ([\d.]+)\s+encoder ([\d.]+)", text)
    if m:
        info["result"].update(mpPlan=float(m[1]), mpGot=float(m[2]))
        info["plan"].setdefault("xFinal", float(m[1]))
    m = re.search(r"time: run ([\d.]+) s \(T ([\d.]+)\)", text)
    if m:
        info["result"].update(runT=float(m[1]), T=float(m[2]))
    return info


def build(csv_path, args):
    samples = read_samples(csv_path)
    if not samples:
        raise SystemExit(f"{csv_path}: no samples")
    info = read_log(os.path.splitext(csv_path)[0] + ".log")
    plan, geom = info["plan"], info["geom"]
    for key, val in (("D", args.D), ("B", args.B), ("xBottle", args.bottle)):
        if val is not None:
            plan[key] = val
    plan.setdefault("D", plan.get("xFinal", 5.0) - plan.get("mpOfs", 0.0))
    geom.setdefault("enc", config_value("ENC_FROM_MP_M") or 0.0)
    geom.setdefault("lp", config_value("L_P_M") or 0.0)
    data = {
        "name": os.path.splitext(os.path.basename(csv_path))[0],
        "samples": samples, "plan": plan, "geom": geom, "result": info["result"],
    }
    return PAGE.replace("__DATA__", json.dumps(data)).replace("__TITLE__", "Run " + data["name"])


def write_plot(csv_path, args=None, open_it=False):
    """Write <csv>.html and return its path. Used by capture_run.py."""
    if args is None:
        args = argparse.Namespace(D=None, B=None, bottle=None)
    out = os.path.splitext(csv_path)[0] + ".html"
    with open(out, "w") as f:
        f.write(build(csv_path, args))
    if open_it and sys.platform == "darwin":
        subprocess.run(["open", out], check=False)
    return out


PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>__TITLE__</title>
<style>
:root {
  color-scheme: light;
  --surface: #fcfcfb; --page: #f4f3f0; --text-1: #0b0b0b; --text-2: #52514e; --muted: #8b8a85;
  --grid: #e6e5e1; --axis: #b9b8b2; --band: #f0efec;
  --series-1: #2a78d6; --series-2: #eb6834; --ref: #52514e;
}
@media (prefers-color-scheme: dark) {
  :root:where(:not([data-theme="light"])) {
    color-scheme: dark;
    --surface: #1a1a19; --page: #111110; --text-1: #f0efec; --text-2: #c3c2b7; --muted: #8b8a85;
    --grid: #2c2c2a; --axis: #4a4a46; --band: #232322;
    --series-1: #3987e5; --series-2: #d95926; --ref: #c3c2b7;
  }
}
:root[data-theme="dark"] {
  color-scheme: dark;
  --surface: #1a1a19; --page: #111110; --text-1: #f0efec; --text-2: #c3c2b7; --muted: #8b8a85;
  --grid: #2c2c2a; --axis: #4a4a46; --band: #232322;
  --series-1: #3987e5; --series-2: #d95926; --ref: #c3c2b7;
}
* { box-sizing: border-box; }
body { margin: 0; background: var(--page); color: var(--text-1);
  font: 14px/1.45 -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; }
main { max-width: 1100px; margin: 0 auto; padding: 20px 16px 40px; }
h1 { font-size: 20px; margin: 0 0 4px; }
.sub { color: var(--text-2); margin: 0 0 16px; }
.tiles { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 10px; margin-bottom: 16px; }
.tile { background: var(--surface); border-radius: 10px; padding: 10px 12px; }
.tile .k { color: var(--text-2); font-size: 12px; }
.tile .v { font-size: 20px; font-variant-numeric: tabular-nums; }
.tile .n { color: var(--muted); font-size: 12px; }
.card { background: var(--surface); border-radius: 12px; padding: 14px 14px 8px; margin-bottom: 14px; position: relative; }
.card h2 { font-size: 15px; margin: 0; }
.card .note { color: var(--text-2); font-size: 12px; margin: 2px 0 6px; }
.legend { display: flex; flex-wrap: wrap; gap: 14px; align-items: center; color: var(--text-2); font-size: 12px; margin: 4px 0; }
.legend .sw { display: inline-block; width: 18px; height: 0; border-top: 2px solid; vertical-align: middle; margin-right: 5px; }
.legend .sw.dash { border-top-style: dashed; }
.ramp { display: inline-block; width: 120px; height: 8px; border-radius: 4px; vertical-align: middle; margin: 0 6px; }
label.tog { cursor: pointer; user-select: none; }
svg { display: block; width: 100%; overflow: visible; }
svg text { fill: var(--text-2); font-size: 11px; font-variant-numeric: tabular-nums; }
svg .lbl { fill: var(--text-1); font-size: 11px; }
.tip { position: absolute; pointer-events: none; background: var(--surface); color: var(--text-1);
  border: 1px solid var(--axis); border-radius: 8px; padding: 6px 8px; font-size: 12px; line-height: 1.35;
  box-shadow: 0 4px 14px rgba(0,0,0,.15); white-space: nowrap; display: none; z-index: 2;
  font-variant-numeric: tabular-nums; }
.tip b { font-weight: 600; }
details { background: var(--surface); border-radius: 12px; padding: 10px 14px; }
summary { cursor: pointer; color: var(--text-2); }
table { border-collapse: collapse; font-size: 12px; font-variant-numeric: tabular-nums; margin-top: 8px; }
th, td { padding: 2px 8px; text-align: right; border-bottom: 1px solid var(--grid); }
th { color: var(--text-2); font-weight: 500; position: sticky; top: 0; background: var(--surface); }
.tablewrap { max-height: 420px; overflow: auto; }
</style></head>
<body><main>
<h1 id="title"></h1>
<p class="sub" id="sub"></p>
<div class="tiles" id="tiles"></div>

<div class="card" id="pathCard">
  <h2>Path the car thinks it drove, top view (rear MP), coloured by speed</h2>
  <p class="note" id="frameNote"></p>
  <div class="legend">
    <span>slow<span class="ramp" id="ramp"></span>fast <span id="vmax"></span></span>
    <span><span class="sw" style="border-color:var(--text-2)"></span>forward</span>
    <span><span class="sw dash" style="border-color:var(--text-2)"></span>reverse</span>
    <label class="tog"><input type="checkbox" id="showRoller"> front roller</label>
  </div>
  <svg id="path"></svg>
</div>

<div class="card"><h2>Calculated position vs time (car's frame)</h2>
  <p class="note">What the car computed from the encoder and the IMU heading, rear MP: x along and y across the direction it pointed at START (heading 0). Not rotated to the floor.</p>
  <div class="legend"><span><span class="sw" style="border-color:var(--series-1)"></span>x, along (m)</span></div>
  <svg id="posx"></svg>
  <div class="legend"><span><span class="sw" style="border-color:var(--series-2)"></span>y, sideways (cm, + = left)</span></div>
  <svg id="posy"></svg></div>

<div class="card"><h2>Speed</h2>
  <div class="legend"><span><span class="sw" style="border-color:var(--series-1)"></span>measured</span>
  <span><span class="sw dash" style="border-color:var(--ref)"></span>target (vref)</span></div>
  <svg id="speed"></svg></div>

<div class="card"><h2>Motor power</h2>
  <svg id="pwm"></svg></div>

<div class="card"><h2>Heading and steering (+ = left)</h2>
  <div class="legend"><span><span class="sw" style="border-color:var(--series-1)"></span>heading (nose)</span>
  <span><span class="sw" style="border-color:var(--series-2)"></span>steer command</span></div>
  <svg id="steer"></svg></div>

<details><summary>Data table</summary><div class="tablewrap"><table id="table"></table></div></details>
<div class="tip" id="tip"></div>
</main>
<script>
const DATA = __DATA__;
const S = DATA.samples, P = DATA.plan, G = DATA.geom, R = DATA.result;
const RAD = Math.PI / 180;
const t0 = S[0].t;
for (const s of S) {
  s.ts = (s.t - t0) / 1000;
  const th = s.th * RAD;
  s.mx = s.rx - G.enc * Math.cos(th);       // rear MP from the roller pose
  s.my = s.ry - G.enc * Math.sin(th);
  s.rev = s.st === "REVERSE" || s.st === "STOP";
}
const NS = "http://www.w3.org/2000/svg";
const el = (tag, attrs, parent) => { const e = document.createElementNS(NS, tag);
  for (const k in attrs) e.setAttribute(k, attrs[k]); if (parent) parent.appendChild(e); return e; };
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
const isDark = () => css("color-scheme") === "dark";
const side = v => (v >= 0 ? "L" : "R");
const fmt = (v, d) => (v >= 0 ? "+" : "") + v.toFixed(d);

// Sequential blue (palette.md): light 250->700, dark 600->100, slow end toward the surface.
const RAMP_L = ["#86b6ef","#6da7ec","#5598e7","#3987e5","#2a78d6","#256abf","#1c5cab","#184f95","#104281","#0d366b"];
const RAMP_D = ["#184f95","#1c5cab","#256abf","#2a78d6","#3987e5","#5598e7","#6da7ec","#86b6ef","#9ec5f4","#cde2fb"];
const hex2 = h => [1,3,5].map(i => parseInt(h.slice(i, i + 2), 16));
function rampColor(f) {
  const r = isDark() ? RAMP_D : RAMP_L;
  f = Math.max(0, Math.min(1, f)) * (r.length - 1);
  const i = Math.min(r.length - 2, Math.floor(f)), u = f - i;
  const a = hex2(r[i]), b = hex2(r[i + 1]);
  return "rgb(" + a.map((c, k) => Math.round(c + (b[k] - c) * u)).join(",") + ")";
}
const VMAX = Math.max(0.1, ...S.map(s => Math.abs(s.v)));
// The car's own frame, as calculated: x along and y across the direction it pointed
// at START (heading 0). cx/cy name the calculated values throughout.
for (const s of S) { s.cx = s.mx; s.cy = s.my; }
document.getElementById("frameNote").innerHTML =
  `What the car <b>thought</b> it did: its own telemetry (encoder + IMU heading), in its own frame. ` +
  (P.aim ? `x is along the direction it pointed at START turned <b>${Math.abs(P.aim).toFixed(2)}° ${P.aim > 0 ? "left" : "right"}</b> by the aim trim, so the path starts angled and steers onto that line. `
         : `x is along and y across the direction it pointed at START (heading 0). `) +
  `An aim error the trim doesn't cover does not show here. Sideways is in cm and stretched.`;

// ---- header ---------------------------------------------------------------------
document.getElementById("title").textContent = "Run " + DATA.name.replace(
  /^(\d{4})(\d\d)(\d\d)-(\d\d)(\d\d)(\d\d)(?:-r(\d+))?$/,
  (m, y, mo, d, h, mi, s, n) => `${y}-${mo}-${d} ${h}:${mi}` + (n ? ` · #${n}` : ""));
const planBits = [];
if (P.D != null) planBits.push("D " + P.D.toFixed(2) + " m");
if (P.B != null) planBits.push("B " + P.B.toFixed(2) + " m");
if (P.T != null) planBits.push("T " + P.T.toFixed(1) + " s");
if (P.xBottle != null) planBits.push("bottle at " + P.xBottle.toFixed(1) + " m");
if (P.btlPast != null) planBits.push("BtlPast " + (100 * P.btlPast).toFixed(1) + " cm");
if (P.aim != null) planBits.push("Aim " + (P.aim >= 0 ? "+" : "") + P.aim.toFixed(2) + "° (track x axis that far left of the START direction)");
document.getElementById("sub").textContent = planBits.join(" · ") || "plan not in the log";
const last = S[S.length - 1];
const tiles = [];
if (R.mpGot != null) tiles.push(["Final MP, along (encoder)", fmt(100 * (R.mpGot - R.mpPlan), 1) + " cm", "planned " + R.mpPlan.toFixed(3) + " m"]);
tiles.push(["Final MP, sideways", Math.abs(100 * last.cy).toFixed(1) + " cm " + (last.cy >= 0 ? "left" : "right"),
            "of the car's own line"]);
if (R.xFgot != null && P.D != null && P.B != null && G.lp) {
  const past = 100 * (R.xFgot + G.lp - (P.D + P.B));   // pusher face at the furthest point
  tiles.push(["Bottle near side (est.)", Math.abs(past).toFixed(1) + " cm " + (past >= 0 ? "past" : "SHORT"),
              "of the Bottle Line" + (P.btlPast != null ? ", aimed " + (100 * P.btlPast).toFixed(1) + " cm" : "")]);
}
if (R.xFgot != null) tiles.push(["Push end vs plan", fmt(100 * (R.xFgot - R.xFplan), 1) + " cm", "xF " + R.xFplan.toFixed(3) + " m"]);
if (R.runT != null) tiles.push(["Run time", R.runT.toFixed(2) + " s", fmt(R.runT - R.T, 2) + " s vs T " + R.T.toFixed(1)]);
document.getElementById("tiles").innerHTML = tiles.map(t =>
  `<div class="tile"><div class="k">${t[0]}</div><div class="v">${t[1]}</div><div class="n">${t[2]}</div></div>`).join("");

// ---- tooltip ----------------------------------------------------------------------
const tip = document.getElementById("tip");
function showTip(card, px, py, s) {
  tip.innerHTML = `<b>${s.st}</b> · t ${s.ts.toFixed(2)} s<br>` +
    `calculated: MP x ${s.cx.toFixed(3)} m · y ${Math.abs(100 * s.cy).toFixed(1)} cm ${side(s.cy)}<br>` +
    `speed ${Math.abs(s.v).toFixed(2)} (target ${s.vref.toFixed(2)}) m/s · power ${s.pwm.toFixed(1)}%<br>` +
    `heading ${Math.abs(s.th).toFixed(2)}° ${side(s.th)} · steer ${Math.abs(s.steer).toFixed(2)}° ${side(s.steer)}` +
    (s.stuck ? "<br>stuck: stall boost" : "") + (s.launch ? "<br>launch kick" : "");
  const r = card.getBoundingClientRect(), m = document.querySelector("main").getBoundingClientRect();
  tip.style.display = "block";
  let x = r.left - m.left + px + 14, y = r.top - m.top + py + 14;
  if (x + tip.offsetWidth > m.width) x = r.left - m.left + px - tip.offsetWidth - 14;
  tip.style.left = x + "px"; tip.style.top = y + "px";
}
const hideTip = () => { tip.style.display = "none"; };

// ---- shared axis helpers ------------------------------------------------------------
function ticks(lo, hi, n) {
  const step0 = (hi - lo) / n, mag = Math.pow(10, Math.floor(Math.log10(step0)));
  const step = [1, 2, 2.5, 5, 10].map(k => k * mag).find(k => k >= step0);
  const out = []; for (let v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) out.push(+v.toFixed(6));
  return out;
}
function frame(svg, W, H, m, xd, yd, xlab, ylab, xfmt, yfmt) {
  const sx = v => m.l + (v - xd[0]) / (xd[1] - xd[0]) * (W - m.l - m.r);
  const sy = v => H - m.b - (v - yd[0]) / (yd[1] - yd[0]) * (H - m.t - m.b);
  const g = el("g", {}, svg);
  for (const v of ticks(yd[0], yd[1], 5)) {
    el("line", {x1: m.l, x2: W - m.r, y1: sy(v), y2: sy(v), stroke: css("--grid")}, g);
    el("text", {x: m.l - 6, y: sy(v) + 4, "text-anchor": "end"}, g).textContent = yfmt(v);
  }
  for (const v of ticks(xd[0], xd[1], Math.max(4, Math.floor((W - m.l - m.r) / 90)))) {
    el("text", {x: sx(v), y: H - m.b + 15, "text-anchor": "middle"}, g).textContent = xfmt(v);
  }
  el("line", {x1: m.l, x2: W - m.r, y1: H - m.b, y2: H - m.b, stroke: css("--axis")}, g);
  el("text", {x: W - m.r, y: H - 4, "text-anchor": "end"}, g).textContent = xlab;
  el("text", {x: 4, y: m.t - 8, "text-anchor": "start"}, g).textContent = ylab;
  return {sx, sy};
}

// ---- path map ------------------------------------------------------------------------
function drawPath() {
  const svg = document.getElementById("path"); svg.innerHTML = "";
  const W = svg.clientWidth || 900, H = Math.max(260, Math.min(380, W * 0.36)), m = {l: 44, r: 16, t: 22, b: 30};
  svg.setAttribute("viewBox", `0 0 ${W} ${H}`); svg.style.height = H + "px";
  const roller = document.getElementById("showRoller").checked;
  const xs = S.map(s => s.mx).concat(roller ? S.map(s => s.rx) : []);
  const xEnd = Math.max(...xs, (P.D || 0) + (P.B || 0)) + 0.25;
  const ys = S.map(s => Math.abs(100 * s.my)).concat(roller ? S.map(s => Math.abs(100 * s.ry)) : []);

  const yl = Math.max(3, Math.ceil(Math.max(...ys) * 1.25));
  const {sx, sy} = frame(svg, W, H, m, [-0.3, xEnd], [-yl, yl], "along the track (m) →",
                         "sideways (cm), + = left", v => v.toFixed(1), v => (v > 0 ? "+" : "") + v);
  const g = el("g", {}, svg);
  el("line", {x1: sx(-0.3), x2: sx(xEnd), y1: sy(0), y2: sy(0), stroke: css("--axis"), "stroke-dasharray": "4 4"}, g);
  const mark = (x, label, dy) => el("text", {x: sx(x), y: sy(0) + dy, "text-anchor": "middle", class: "lbl"}, g).textContent = label;
  el("circle", {cx: sx(0), cy: sy(0), r: 4, fill: css("--text-2")}, g); mark(0, "Start", -10);
  if (P.xBottle != null) { el("circle", {cx: sx(P.xBottle), cy: sy(0), r: 6, fill: "none", stroke: css("--text-2"), "stroke-width": 1.5}, g); mark(P.xBottle, "bottle", 22); }
  if (P.D != null) {
    const x = sx(P.D), y = sy(0);
    el("path", {d: `M${x-5},${y-5}L${x+5},${y+5}M${x-5},${y+5}L${x+5},${y-5}`, stroke: css("--text-1"), "stroke-width": 2}, g);
    mark(P.D, "Target", 22);
  }
  if (P.D != null && P.B != null) {
    const x = sx(P.D + P.B);
    el("line", {x1: x, x2: x, y1: m.t, y2: H - m.b, stroke: css("--muted"), "stroke-width": 2}, g);
    el("text", {x: x + 4, y: m.t + 10, class: "lbl"}, g).textContent = "Bottle Line";
  }
  if (roller) {
    el("polyline", {points: S.map(s => sx(s.rx) + "," + sy(100 * s.ry)).join(" "), fill: "none",
                    stroke: css("--muted"), "stroke-width": 1}, g);
  }
  for (let i = 1; i < S.length; i++) {
    const a = S[i - 1], b = S[i];
    el("line", {x1: sx(a.mx), y1: sy(100 * a.my), x2: sx(b.mx), y2: sy(100 * b.my),
                stroke: rampColor((Math.abs(a.v) + Math.abs(b.v)) / 2 / VMAX), "stroke-width": 3,
                "stroke-linecap": "round", "stroke-dasharray": b.rev ? "6 5" : "none"}, g);
  }
  // Push end: the pusher face (bottle near side) when the car stopped forward.
  const turn = S.find(s => s.st === "TURN");
  if (turn && G.lp) {
    // The car's body at the push end: the path is its REAR (the MP); the pusher, with
    // the bottle against it, is L_P_M ahead. Drawn behind the path.
    const x = turn.mx + G.lp;
    const y = sy(100 * turn.my);
    const body = el("line", {x1: sx(turn.mx), x2: sx(x), y1: y, y2: y, stroke: css("--muted"),
                             "stroke-width": 10, "stroke-linecap": "butt", opacity: 0.35}, g);
    g.insertBefore(body, g.firstChild);
    el("text", {x: (sx(turn.mx) + sx(x)) / 2, y: y - 10, "text-anchor": "middle", class: "lbl"}, g).textContent = "car at the push end";
    el("line", {x1: sx(x), x2: sx(x), y1: y - 14, y2: y + 14, stroke: css("--series-2"), "stroke-width": 2}, g);
    el("text", {x: sx(x) - 6, y: y + 28, "text-anchor": "end", class: "lbl"}, g).textContent = "pusher / bottle near side";
  }
  el("circle", {cx: sx(last.mx), cy: sy(100 * last.my), r: 5, fill: css("--text-1"), stroke: css("--surface"), "stroke-width": 2}, g);
  el("text", {x: sx(last.mx) - 10, y: sy(100 * last.my) + 4, "text-anchor": "end", class: "lbl"}, g).textContent =
    "final MP";
  const hit = el("rect", {x: 0, y: 0, width: W, height: H, fill: "transparent"}, svg);
  const card = document.getElementById("pathCard");
  hit.addEventListener("mousemove", e => {
    const r = svg.getBoundingClientRect(), px = e.clientX - r.left, py = e.clientY - r.top;
    let best = null, bd = 1e9;
    for (const s of S) { const d = Math.hypot(sx(s.mx) - px, sy(100 * s.my) - py); if (d < bd) { bd = d; best = s; } }
    if (best && bd < 40) showTip(card, sx(best.mx), sy(100 * best.my), best); else hideTip();
  });
  hit.addEventListener("mouseleave", hideTip);
  document.getElementById("ramp").style.background =
    "linear-gradient(90deg," + Array.from({length: 11}, (_, i) => rampColor(i / 10)).join(",") + ")";
  document.getElementById("vmax").textContent = "(0–" + VMAX.toFixed(2) + " m/s)";
}

// ---- time charts --------------------------------------------------------------------
const STATES = ["CRUISE", "CAPTURE", "PUSH", "TURN", "REVERSE"];
function timeChart(id, series, ylab, yfmt, zeroLine) {
  const svg = document.getElementById(id); svg.innerHTML = "";
  const W = svg.clientWidth || 900, H = 196, m = {l: 44, r: 16, t: 30, b: 30};
  svg.setAttribute("viewBox", `0 0 ${W} ${H}`); svg.style.height = H + "px";
  const vals = series.flatMap(sr => S.map(sr.get));
  let lo = Math.min(0, ...vals), hi = Math.max(...vals);
  const pad = (hi - lo) * 0.08 || 1; lo = lo < 0 ? lo - pad : 0; hi += pad;
  const tEnd = S[S.length - 1].ts;
  const {sx, sy} = frame(svg, W, H, m, [0, tEnd], [lo, hi], "time since START (s) →", ylab,
                         v => v.toFixed(0), yfmt);
  const bands = el("g", {}, svg); svg.insertBefore(bands, svg.firstChild);
  let start = 0, k = 0;
  for (let i = 1; i <= S.length; i++) {
    if (i === S.length || S[i].st !== S[start].st) {
      const x1 = sx(S[start].ts), x2 = sx(i === S.length ? tEnd : S[i].ts);
      if (k % 2 === 0) el("rect", {x: x1, y: m.t, width: Math.max(0, x2 - x1), height: H - m.t - m.b, fill: css("--band")}, bands);
      if (x2 - x1 > 34) el("text", {x: (x1 + x2) / 2, y: m.t - 8, "text-anchor": "middle"}, bands).textContent = S[start].st.toLowerCase();
      start = i; k++;
    }
  }
  if (zeroLine) el("line", {x1: m.l, x2: W - m.r, y1: sy(0), y2: sy(0), stroke: css("--axis")}, svg);
  for (const sr of series) {
    el("polyline", {points: S.map(s => sx(s.ts) + "," + sy(sr.get(s))).join(" "), fill: "none",
                    stroke: css(sr.color), "stroke-width": 2, "stroke-dasharray": sr.dash ? "5 4" : "none",
                    "stroke-linejoin": "round"}, svg);
  }
  const cross = el("line", {y1: m.t, y2: H - m.b, stroke: css("--axis"), visibility: "hidden"}, svg);
  const hit = el("rect", {x: m.l, y: 0, width: W - m.l - m.r, height: H, fill: "transparent"}, svg);
  const card = svg.closest(".card");
  hit.addEventListener("mousemove", e => {
    const r = svg.getBoundingClientRect(), px = e.clientX - r.left;
    let best = S[0]; for (const s of S) if (Math.abs(sx(s.ts) - px) < Math.abs(sx(best.ts) - px)) best = s;
    cross.setAttribute("x1", sx(best.ts)); cross.setAttribute("x2", sx(best.ts)); cross.setAttribute("visibility", "visible");
    showTip(card, sx(best.ts), e.clientY - r.top, best);
  });
  hit.addEventListener("mouseleave", () => { cross.setAttribute("visibility", "hidden"); hideTip(); });
}

function drawAll() {
  drawPath();
  timeChart("posx", [{get: s => s.cx, color: "--series-1"}], "m", v => v.toFixed(1), false);
  timeChart("posy", [{get: s => 100 * s.cy, color: "--series-2"}], "cm", v => (v > 0 ? "+" : "") + v.toFixed(1), true);
  timeChart("speed", [{get: s => Math.abs(s.vref), color: "--ref", dash: true},
                      {get: s => Math.abs(s.v), color: "--series-1"}], "m/s", v => v.toFixed(1), false);
  timeChart("pwm", [{get: s => s.pwm, color: "--series-1"}], "%", v => v.toFixed(0), false);
  timeChart("steer", [{get: s => s.th, color: "--series-1"}, {get: s => s.steer, color: "--series-2"}],
            "degrees", v => (v > 0 ? "+" : "") + v.toFixed(1), true);
}

// ---- table ----------------------------------------------------------------------------
const cols = [["t (s)", s => s.ts.toFixed(2)], ["state", s => s.st], ["calc x (m)", s => s.cx.toFixed(3)],
  ["calc y (cm)", s => (100 * s.cy).toFixed(1)], ["speed", s => s.v.toFixed(3)], ["target", s => s.vref.toFixed(3)],
  ["power %", s => s.pwm.toFixed(1)], ["heading °", s => s.th.toFixed(2)], ["steer °", s => s.steer.toFixed(2)]];
document.getElementById("table").innerHTML = "<tr>" + cols.map(c => `<th>${c[0]}</th>`).join("") + "</tr>" +
  S.map(s => "<tr>" + cols.map(c => `<td>${c[1](s)}</td>`).join("") + "</tr>").join("");

drawAll();
document.getElementById("showRoller").addEventListener("change", drawPath);
new ResizeObserver(() => drawAll()).observe(document.querySelector("main"));
matchMedia("(prefers-color-scheme: dark)").addEventListener("change", drawAll);
</script></body></html>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+", help="run CSV(s) from run_logs/")
    ap.add_argument("--open", action="store_true", help="open the page in the browser (macOS)")
    ap.add_argument("--D", type=float, help="Target Distance, m (if the log has no PLAN line)")
    ap.add_argument("--B", type=float, help="Bottle Line Distance, m (if the log has no PLAN line)")
    ap.add_argument("--bottle", type=float, help="bottle centre from Start, m (if the log has no PLAN line)")
    args = ap.parse_args()
    for path in args.csv:
        print(write_plot(path, args, open_it=args.open))


if __name__ == "__main__":
    main()
