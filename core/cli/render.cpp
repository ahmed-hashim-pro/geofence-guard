#include "render.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>

namespace gfg_cli {

namespace {

constexpr double kWidth = 760.0;   // drawing area, px
constexpr double kPad = 36.0;      // around the drawing area
constexpr double kHeader = 34.0;   // title line
constexpr double kFooter = 64.0;   // legend and scale bar

const char* const kOk = "#1a7f37";
const char* const kRefused = "#cf222e";
const char* const kRecovery = "#bc4c00";
const char* const kBand = "#d4a72c";
const char* const kInk = "#24292f";
const char* const kMuted = "#57606a";

std::string num(double v) {
  char buf[32];
  std::snprintf(buf, sizeof buf, "%.1f", v);
  return buf;
}

std::string xml_escape(const std::string& s) {
  std::string out;
  for (const char c : s) {
    switch (c) {
      case '&': out += "&amp;"; break;
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '"': out += "&quot;"; break;
      default: out += c;
    }
  }
  return out;
}

// World metres to SVG pixels, north up.
struct View {
  double min_x, max_y, scale;
  double px(double x) const { return kPad + (x - min_x) * scale; }
  double py(double y) const { return kPad + kHeader + (max_y - y) * scale; }
};

std::string points(const gfg::Polygon& poly, const View& v) {
  std::string s;
  for (const auto& p : poly) s += num(v.px(p.x)) + "," + num(v.py(p.y)) + " ";
  return s;
}

const char* leg_colour(const gfg::Decision& d) {
  if (!d.accepted) return kRefused;
  return d.recovery ? kRecovery : kOk;
}

}  // namespace

std::vector<Leg> decide_legs(const gfg::Guard& guard, const std::vector<gfg::Vec3>& mission) {
  std::vector<Leg> legs;
  const auto& frame = guard.envelope().frame;
  for (std::size_t i = 1; i < mission.size(); ++i) {
    legs.push_back({mission[i - 1], mission[i],
                    guard.check({frame, 0.0, mission[i - 1]}, {frame, 0.0, mission[i]})});
  }
  return legs;
}

std::string render_svg(const gfg::Envelope& env, const std::vector<Leg>& legs,
                       const std::string& title) {
  // Bounds: the fence, every waypoint, and the margin around both.
  double min_x = INFINITY, max_x = -INFINITY, min_y = INFINITY, max_y = -INFINITY;
  const auto grow = [&](double x, double y) {
    min_x = std::min(min_x, x);
    max_x = std::max(max_x, x);
    min_y = std::min(min_y, y);
    max_y = std::max(max_y, y);
  };
  for (const auto& p : env.fence) grow(p.x, p.y);
  for (const auto& l : legs) {
    grow(l.from.x, l.from.y);
    grow(l.to.x, l.to.y);
  }
  const double slack = std::max(env.margin * 2.0, 0.04 * std::max(max_x - min_x, max_y - min_y));
  min_x -= slack;
  max_x += slack;
  min_y -= slack;
  max_y += slack;

  const View v{min_x, max_y, kWidth / (max_x - min_x)};
  const double height = (max_y - min_y) * v.scale;
  const double total_w = kWidth + 2 * kPad;
  const double total_h = height + 2 * kPad + kHeader + kFooter;
  const double band = 2.0 * env.margin * v.scale;  // stroke straddles the edge

  std::ostringstream o;
  o << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << num(total_w) << "\" height=\""
    << num(total_h) << "\" viewBox=\"0 0 " << num(total_w) << " " << num(total_h)
    << "\" font-family=\"-apple-system, 'Segoe UI', Helvetica, Arial, sans-serif\">\n";
  o << "<defs>\n"
    ;
  for (const char* c : {kOk, kRefused, kRecovery}) {
    o << "  <marker id=\"arrow" << (c + 1) << "\" viewBox=\"0 0 10 10\" refX=\"9\" refY=\"5\" "
      << "markerWidth=\"7\" markerHeight=\"7\" orient=\"auto-start-reverse\">"
      << "<path d=\"M0,0 L10,5 L0,10 z\" fill=\"" << c << "\"/></marker>\n";
  }
  o << "</defs>\n";
  o << "<rect width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>\n";

  // Fence, then its margin band: a stroke 2*margin wide with round joins is
  // exactly the set of points within `margin` of the boundary; clipping it to
  // the fence leaves the inner half, which is the part a target cannot use.
  o << "<polygon points=\"" << points(env.fence, v) << "\" fill=\"#f0f7f0\"/>\n";
  o << "<polygon points=\"" << points(env.fence, v) << "\" fill=\"none\" stroke=\"" << kBand
    << "\" stroke-opacity=\"0.45\" stroke-width=\"" << num(band) << "\" stroke-linejoin=\"round\"/>\n";
  // White out the half of the band that lies outside the fence: the page
  // rectangle with the fence cut out of it. Works in renderers without clipPath.
  o << "<path fill=\"#ffffff\" fill-rule=\"evenodd\" d=\"M0,0 H" << num(total_w) << " V"
    << num(total_h) << " H0 Z M" << points(env.fence, v) << "Z\"/>\n";
  o << "<polygon points=\"" << points(env.fence, v) << "\" fill=\"none\" stroke=\"" << kInk
    << "\" stroke-width=\"1.5\"/>\n";

  o << "<text x=\"" << num(kPad) << "\" y=\"" << num(kPad + 4) << "\" font-size=\"16\" "
    << "font-weight=\"600\" fill=\"" << kInk << "\">" << xml_escape(title) << "</text>\n";
  for (const auto& z : env.exclusions) {
    o << "<polygon points=\"" << points(z.polygon, v) << "\" fill=\"none\" stroke=\"" << kBand
      << "\" stroke-opacity=\"0.45\" stroke-width=\"" << num(band) << "\" stroke-linejoin=\"round\"/>\n";
    o << "<polygon points=\"" << points(z.polygon, v) << "\" fill=\"#ffebe9\" stroke=\"" << kRefused
      << "\" stroke-width=\"1.5\"/>\n";
    double cx = 0, cy = 0;
    for (const auto& p : z.polygon) {
      cx += p.x;
      cy += p.y;
    }
    cx /= static_cast<double>(z.polygon.size());
    cy /= static_cast<double>(z.polygon.size());
    o << "<text x=\"" << num(v.px(cx)) << "\" y=\"" << num(v.py(cy) - band / 2 - 14)
      << "\" font-size=\"12\" text-anchor=\"middle\" fill=\"" << kRefused << "\">"
      << xml_escape(z.name) << "</text>\n";
  }

  // Legs, numbered at their midpoints; refused ones dashed and annotated.
  for (std::size_t i = 0; i < legs.size(); ++i) {
    const auto& l = legs[i];
    const char* c = leg_colour(l.decision);
    o << "<line x1=\"" << num(v.px(l.from.x)) << "\" y1=\"" << num(v.py(l.from.y)) << "\" x2=\""
      << num(v.px(l.to.x)) << "\" y2=\"" << num(v.py(l.to.y)) << "\" stroke=\"" << c
      << "\" stroke-width=\"2.5\"" << (l.decision.accepted ? "" : " stroke-dasharray=\"8 5\"")
      << " marker-end=\"url(#arrow" << (c + 1) << ")\"/>\n";
    const double mx = v.px((l.from.x + l.to.x) / 2);
    const double my = v.py((l.from.y + l.to.y) / 2);
    o << "<circle cx=\"" << num(mx) << "\" cy=\"" << num(my) << "\" r=\"9\" fill=\"#ffffff\" "
      << "stroke=\"" << c << "\" stroke-width=\"1.5\"/>\n";
    o << "<text x=\"" << num(mx) << "\" y=\"" << num(my + 4) << "\" font-size=\"11\" "
      << "font-weight=\"600\" text-anchor=\"middle\" fill=\"" << c << "\">" << (i + 1) << "</text>\n";
    if (!l.decision.accepted) {
      double ty = my + 26;
      for (const auto& viol : l.decision.violations) {
        const std::string label = std::string("REFUSED: ") + gfg::reason_code(viol.reason);
        o << "<rect x=\"" << num(mx + 10) << "\" y=\"" << num(ty - 13) << "\" width=\""
          << num(8 + 7.2 * static_cast<double>(label.size())) << "\" height=\"18\" rx=\"3\" "
          << "fill=\"#ffffff\" stroke=\"" << kRefused << "\" stroke-width=\"1\"/>\n";
        o << "<text x=\"" << num(mx + 14) << "\" y=\"" << num(ty) << "\" font-size=\"12\" "
          << "font-weight=\"600\" fill=\"" << kRefused << "\">" << label << "</text>\n";
        ty += 22;
      }
    }
  }
  if (!legs.empty()) {
    const auto& s = legs.front().from;
    o << "<rect x=\"" << num(v.px(s.x) - 6) << "\" y=\"" << num(v.py(s.y) - 6)
      << "\" width=\"12\" height=\"12\" fill=\"" << kInk << "\"/>\n";
    o << "<text x=\"" << num(v.px(s.x) + 10) << "\" y=\"" << num(v.py(s.y) + 16)
      << "\" font-size=\"11\" fill=\"" << kInk << "\">start</text>\n";
  }

  // Legend and a scale bar.
  const double ly = kPad + kHeader + height + 30;
  double lx = kPad;
  const auto swatch_line = [&](const char* colour, bool dashed, const std::string& label) {
    o << "<line x1=\"" << num(lx) << "\" y1=\"" << num(ly) << "\" x2=\"" << num(lx + 26)
      << "\" y2=\"" << num(ly) << "\" stroke=\"" << colour << "\" stroke-width=\"2.5\""
      << (dashed ? " stroke-dasharray=\"8 5\"" : "") << "/>\n";
    o << "<text x=\"" << num(lx + 32) << "\" y=\"" << num(ly + 4) << "\" font-size=\"12\" fill=\""
      << kMuted << "\">" << label << "</text>\n";
    lx += 40 + 6.5 * static_cast<double>(label.size());
  };
  swatch_line(kOk, false, "accepted");
  swatch_line(kRecovery, false, "accepted (recovery)");
  swatch_line(kRefused, true, "refused");
  o << "<rect x=\"" << num(lx) << "\" y=\"" << num(ly - 6) << "\" width=\"26\" height=\"12\" fill=\""
    << kBand << "\" fill-opacity=\"0.45\"/>\n";
  const std::string band_label = "margin band (" + num(env.margin) + " m)";
  o << "<text x=\"" << num(lx + 32) << "\" y=\"" << num(ly + 4) << "\" font-size=\"12\" fill=\""
    << kMuted << "\">" << band_label << "</text>\n";

  // Scale bar: the largest of 10/20/50/100/200/500 m that fits in 140 px.
  double bar_m = 10;
  for (const double m : {20.0, 50.0, 100.0, 200.0, 500.0, 1000.0}) {
    if (m * v.scale <= 140) bar_m = m;
  }
  const double bx = kPad + kWidth - bar_m * v.scale;
  const double by = ly + 26;
  o << "<path d=\"M" << num(bx) << "," << num(by - 4) << " v4 h" << num(bar_m * v.scale)
    << " v-4\" fill=\"none\" stroke=\"" << kInk << "\" stroke-width=\"1.5\"/>\n";
  o << "<text x=\"" << num(bx + bar_m * v.scale / 2) << "\" y=\"" << num(by - 8)
    << "\" font-size=\"11\" text-anchor=\"middle\" fill=\"" << kMuted << "\">"
    << static_cast<int>(bar_m) << " m</text>\n";
  o << "<text x=\"" << num(kPad) << "\" y=\"" << num(by) << "\" font-size=\"11\" fill=\""
    << kMuted << "\">frame '" << xml_escape(env.frame) << "', north up, altitude band "
    << num(env.min_altitude) << "-" << num(env.max_altitude) << " m</text>\n";

  o << "</svg>\n";
  return o.str();
}

}  // namespace gfg_cli
