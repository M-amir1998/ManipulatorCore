#include "manipulatorcore/collision/distance.hpp"

#include "manipulatorcore/math/error.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace mc {

// ------------------------------------------------------------------------------------------------
// Elementary queries
// ------------------------------------------------------------------------------------------------

double closestPointsSegmentSegment(const Vec3& p1, const Vec3& q1, const Vec3& p2, const Vec3& q2, Vec3& c1,
                                   Vec3& c2) {
  constexpr double kEps = 1e-18;
  const Vec3 d1 = q1 - p1;
  const Vec3 d2 = q2 - p2;
  const Vec3 r = p1 - p2;
  const double a = d1.squaredNorm();
  const double e = d2.squaredNorm();
  const double f = d2.dot(r);
  double s = 0.0;
  double t = 0.0;
  if (a <= kEps && e <= kEps) {
    s = 0.0;
    t = 0.0;
  } else if (a <= kEps) {
    t = std::clamp(f / e, 0.0, 1.0);
  } else {
    const double c = d1.dot(r);
    if (e <= kEps) {
      s = std::clamp(-c / a, 0.0, 1.0);
    } else {
      const double b = d1.dot(d2);
      const double denom = a * e - b * b;  // >= 0; zero for parallel segments
      s = denom > kEps * a * e ? std::clamp((b * f - c * e) / denom, 0.0, 1.0) : 0.0;
      t = (b * s + f) / e;
      if (t < 0.0) {
        t = 0.0;
        s = std::clamp(-c / a, 0.0, 1.0);
      } else if (t > 1.0) {
        t = 1.0;
        s = std::clamp((b - c) / a, 0.0, 1.0);
      }
    }
  }
  c1 = p1 + s * d1;
  c2 = p2 + t * d2;
  return (c1 - c2).squaredNorm();
}

double signedDistancePointBox(const Vec3& point, const SE3& T_box, const Vec3& h, Vec3* closest) {
  const Vec3 local = T_box.rotation().transpose() * (point - T_box.translation());
  const Vec3 q = local.cwiseAbs() - h;
  if (q.maxCoeff() > 0.0) {
    const Vec3 clamped = local.cwiseMax(-h).cwiseMin(h);
    if (closest != nullptr) {
      *closest = T_box * clamped;
    }
    return (local - clamped).norm();
  }
  Index k = 0;
  const double inside = q.maxCoeff(&k);  // distance to the nearest face (negative)
  if (closest != nullptr) {
    Vec3 on_face = local;
    on_face(k) = local(k) >= 0.0 ? h(k) : -h(k);
    *closest = T_box * on_face;
  }
  return inside;
}

// ------------------------------------------------------------------------------------------------
// GJK
// ------------------------------------------------------------------------------------------------

namespace {

struct SimplexVertex {
  Vec3 w;  // a - b
  Vec3 a;
  Vec3 b;
};

struct SimplexClosest {
  Vec3 point;
  std::array<double, 4> lambda{};
  std::vector<int> support;  // indices of the vertices with non-zero weight
};

SimplexClosest closestOnSegment(const std::vector<SimplexVertex>& S, int i0, int i1) {
  const Vec3& A = S[static_cast<std::size_t>(i0)].w;
  const Vec3 AB = S[static_cast<std::size_t>(i1)].w - A;
  const double denom = AB.squaredNorm();
  const double t = denom > 0.0 ? std::clamp(-A.dot(AB) / denom, 0.0, 1.0) : 0.0;
  SimplexClosest out;
  out.point = A + t * AB;
  if (t <= 0.0) {
    out.lambda[static_cast<std::size_t>(i0)] = 1.0;
    out.support = {i0};
  } else if (t >= 1.0) {
    out.lambda[static_cast<std::size_t>(i1)] = 1.0;
    out.support = {i1};
  } else {
    out.lambda[static_cast<std::size_t>(i0)] = 1.0 - t;
    out.lambda[static_cast<std::size_t>(i1)] = t;
    out.support = {i0, i1};
  }
  return out;
}

// Closest point of triangle (i0, i1, i2) to the origin (Ericson, section 5.1.5).
SimplexClosest closestOnTriangle(const std::vector<SimplexVertex>& S, int i0, int i1, int i2) {
  const Vec3& a = S[static_cast<std::size_t>(i0)].w;
  const Vec3& b = S[static_cast<std::size_t>(i1)].w;
  const Vec3& c = S[static_cast<std::size_t>(i2)].w;
  const Vec3 ab = b - a;
  const Vec3 ac = c - a;
  SimplexClosest out;
  auto vertex = [&](int i, const Vec3& p) {
    out.point = p;
    out.lambda[static_cast<std::size_t>(i)] = 1.0;
    out.support = {i};
    return out;
  };
  auto edge = [&](int i, int j, const Vec3& p, const Vec3& q, double t) {
    out.point = p + t * (q - p);
    out.lambda[static_cast<std::size_t>(i)] = 1.0 - t;
    out.lambda[static_cast<std::size_t>(j)] = t;
    out.support = {i, j};
    return out;
  };
  const Vec3 ap = -a;
  const double d1 = ab.dot(ap);
  const double d2 = ac.dot(ap);
  if (d1 <= 0.0 && d2 <= 0.0) {
    return vertex(i0, a);
  }
  const Vec3 bp = -b;
  const double d3 = ab.dot(bp);
  const double d4 = ac.dot(bp);
  if (d3 >= 0.0 && d4 <= d3) {
    return vertex(i1, b);
  }
  const double vc = d1 * d4 - d3 * d2;
  if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
    return edge(i0, i1, a, b, d1 / (d1 - d3));
  }
  const Vec3 cp = -c;
  const double d5 = ab.dot(cp);
  const double d6 = ac.dot(cp);
  if (d6 >= 0.0 && d5 <= d6) {
    return vertex(i2, c);
  }
  const double vb = d5 * d2 - d1 * d6;
  if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
    return edge(i0, i2, a, c, d2 / (d2 - d6));
  }
  const double va = d3 * d6 - d5 * d4;
  if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0) {
    return edge(i1, i2, b, c, (d4 - d3) / ((d4 - d3) + (d5 - d6)));
  }
  const double denom = 1.0 / (va + vb + vc);
  const double v = vb * denom;
  const double w = vc * denom;
  out.point = a + v * ab + w * ac;
  out.lambda[static_cast<std::size_t>(i0)] = 1.0 - v - w;
  out.lambda[static_cast<std::size_t>(i1)] = v;
  out.lambda[static_cast<std::size_t>(i2)] = w;
  out.support = {i0, i1, i2};
  return out;
}

/// True if the origin and vertex d lie on opposite sides of the plane through (a, b, c).
bool originOutsideFace(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d) {
  const Vec3 n = (b - a).cross(c - a);
  const double sign_origin = (-a).dot(n);
  const double sign_d = (d - a).dot(n);
  return sign_origin * sign_d < 0.0;
}

bool closestOnSimplex(std::vector<SimplexVertex>& S, Vec3& v, std::array<double, 4>& lambda) {
  SimplexClosest best;
  switch (S.size()) {
    case 1:
      best.point = S[0].w;
      best.lambda[0] = 1.0;
      best.support = {0};
      break;
    case 2:
      best = closestOnSegment(S, 0, 1);
      break;
    case 3:
      best = closestOnTriangle(S, 0, 1, 2);
      break;
    default: {
      const std::array<std::array<int, 4>, 4> faces = {{{0, 1, 2, 3}, {0, 1, 3, 2}, {0, 2, 3, 1}, {1, 2, 3, 0}}};
      bool any_outside = false;
      double best_sq = std::numeric_limits<double>::infinity();
      for (const auto& f : faces) {
        if (!originOutsideFace(S[static_cast<std::size_t>(f[0])].w, S[static_cast<std::size_t>(f[1])].w,
                               S[static_cast<std::size_t>(f[2])].w, S[static_cast<std::size_t>(f[3])].w)) {
          continue;
        }
        any_outside = true;
        SimplexClosest c = closestOnTriangle(S, f[0], f[1], f[2]);
        if (c.point.squaredNorm() < best_sq) {
          best_sq = c.point.squaredNorm();
          best = c;
        }
      }
      if (!any_outside) {
        return true;  // origin inside the tetrahedron: the sets intersect
      }
      break;
    }
  }
  std::vector<SimplexVertex> reduced;
  std::array<double, 4> reduced_lambda{};
  for (std::size_t k = 0; k < best.support.size(); ++k) {
    const auto idx = static_cast<std::size_t>(best.support[k]);
    reduced.push_back(S[idx]);
    reduced_lambda[k] = best.lambda[idx];
  }
  S = std::move(reduced);
  lambda = reduced_lambda;
  v = best.point;
  return false;
}

}  // namespace

GjkResult gjkDistance(const SupportFunction& support_a, const SupportFunction& support_b,
                      const Vec3& initial_direction, int max_iterations) {
  constexpr double kRelTol = 1e-12;
  constexpr double kAbsTolSq = 1e-24;
  GjkResult result;
  const Vec3 d0 = initial_direction.squaredNorm() > 0.0 ? initial_direction : Vec3::UnitX();
  std::vector<SimplexVertex> S;
  std::array<double, 4> lambda{};
  Vec3 a0 = support_a(-d0);
  Vec3 b0 = support_b(d0);
  Vec3 v = a0 - b0;
  S.push_back({v, a0, b0});
  lambda[0] = 1.0;
  for (int it = 0; it < max_iterations; ++it) {
    result.iterations = it + 1;
    const double vv = v.squaredNorm();
    if (vv <= kAbsTolSq) {
      result.intersecting = true;
      break;
    }
    const Vec3 a = support_a(-v);
    const Vec3 b = support_b(v);
    const Vec3 w = a - b;
    // Converged when the support point does not move the lower bound appreciably.
    if (vv - v.dot(w) <= kRelTol * vv) {
      break;
    }
    const bool duplicate =
        std::any_of(S.begin(), S.end(), [&](const SimplexVertex& s) { return (s.w - w).squaredNorm() <= kAbsTolSq; });
    if (duplicate) {
      break;
    }
    S.push_back({w, a, b});
    if (closestOnSimplex(S, v, lambda)) {
      result.intersecting = true;
      break;
    }
  }
  if (result.intersecting) {
    result.distance = 0.0;
    return result;
  }
  result.point_a.setZero();
  result.point_b.setZero();
  for (std::size_t k = 0; k < S.size(); ++k) {
    result.point_a += lambda[k] * S[k].a;
    result.point_b += lambda[k] * S[k].b;
  }
  result.distance = v.norm();
  return result;
}

// ------------------------------------------------------------------------------------------------
// Primitive pairs
// ------------------------------------------------------------------------------------------------

namespace {

/// A primitive reduced to a convex core (point, segment, box or plane) plus a radius.
struct Core {
  enum class Type { Point, Segment, Box, Plane };
  Type type = Type::Point;
  Vec3 p0 = Vec3::Zero();  // point, or segment start
  Vec3 p1 = Vec3::Zero();  // segment end (== p0 for a point)
  double radius = 0.0;
  SE3 pose;                // box pose
  Vec3 h = Vec3::Zero();   // box half extents
  Vec3 n = Vec3::UnitZ();  // plane normal (world)
  double d = 0.0;          // plane offset: solid where n . x <= d
};

Core makeCore(const ShapeGeometry& g, const SE3& T) {
  Core c;
  if (const auto* s = std::get_if<Sphere>(&g)) {
    c.type = Core::Type::Point;
    c.p0 = c.p1 = T.translation();
    c.radius = s->radius;
  } else if (const auto* cap = std::get_if<Capsule>(&g)) {
    c.type = Core::Type::Segment;
    c.p0 = T * Vec3(0.0, 0.0, -cap->half_length);
    c.p1 = T * Vec3(0.0, 0.0, cap->half_length);
    c.radius = cap->radius;
  } else if (const auto* box = std::get_if<Box>(&g)) {
    c.type = Core::Type::Box;
    c.pose = T;
    c.h = box->half_extents;
  } else {
    const auto& hs = std::get<HalfSpace>(g);
    c.type = Core::Type::Plane;
    c.n = T.rotation() * hs.normal;
    c.d = hs.offset + c.n.dot(T.translation());
  }
  return c;
}

Vec3 coreSupport(const Core& c, const Vec3& dir) {
  switch (c.type) {
    case Core::Type::Point:
      return c.p0;
    case Core::Type::Segment:
      return dir.dot(c.p1 - c.p0) > 0.0 ? c.p1 : c.p0;
    case Core::Type::Box: {
      const Vec3 local = c.pose.rotation().transpose() * dir;
      const Vec3 corner(local.x() >= 0.0 ? c.h.x() : -c.h.x(), local.y() >= 0.0 ? c.h.y() : -c.h.y(),
                        local.z() >= 0.0 ? c.h.z() : -c.h.z());
      return c.pose * corner;
    }
    case Core::Type::Plane:
      break;
  }
  throw InvalidArgument("coreSupport: planes are unbounded");
}

Vec3 coreCenter(const Core& c) {
  return c.type == Core::Type::Box ? Vec3(c.pose.translation()) : Vec3(0.5 * (c.p0 + c.p1));
}

DistanceResult flipped(DistanceResult r) {
  std::swap(r.point_a, r.point_b);
  r.normal = -r.normal;
  return r;
}

Vec3 fallbackNormal(const Core& a, const Core& b) {
  const Vec3 d = coreCenter(b) - coreCenter(a);
  return d.norm() > 1e-12 ? Vec3(d.normalized()) : Vec3(Vec3::UnitZ());
}

DistanceResult roundedVsRounded(const Core& a, const Core& b) {
  Vec3 c1;
  Vec3 c2;
  const double core = std::sqrt(closestPointsSegmentSegment(a.p0, a.p1, b.p0, b.p1, c1, c2));
  DistanceResult r;
  r.normal = core > 1e-12 ? Vec3((c2 - c1) / core) : fallbackNormal(a, b);
  r.distance = core - a.radius - b.radius;
  r.point_a = c1 + a.radius * r.normal;
  r.point_b = c2 - b.radius * r.normal;
  return r;
}

/// Rounded point/segment A against box B: minimizes the (convex) box signed-distance function along
/// the core segment with golden-section search.
DistanceResult roundedVsBox(const Core& a, const Core& b) {
  const auto f = [&](double t) { return signedDistancePointBox(a.p0 + t * (a.p1 - a.p0), b.pose, b.h); };
  double t_best = 0.0;
  if ((a.p1 - a.p0).squaredNorm() > 0.0) {
    constexpr double kInvPhi = 0.6180339887498949;
    double lo = 0.0;
    double hi = 1.0;
    double x1 = hi - kInvPhi * (hi - lo);
    double x2 = lo + kInvPhi * (hi - lo);
    double f1 = f(x1);
    double f2 = f(x2);
    for (int it = 0; it < 80 && hi - lo > 1e-13; ++it) {
      if (f1 <= f2) {
        hi = x2;
        x2 = x1;
        f2 = f1;
        x1 = hi - kInvPhi * (hi - lo);
        f1 = f(x1);
      } else {
        lo = x1;
        x1 = x2;
        f1 = f2;
        x2 = lo + kInvPhi * (hi - lo);
        f2 = f(x2);
      }
    }
    t_best = 0.5 * (lo + hi);
    // The minimum of a convex function may sit at an endpoint.
    if (f(0.0) < f(t_best)) {
      t_best = 0.0;
    }
    if (f(1.0) < f(t_best)) {
      t_best = 1.0;
    }
  }
  const Vec3 c = a.p0 + t_best * (a.p1 - a.p0);
  Vec3 q;
  const double sd = signedDistancePointBox(c, b.pose, b.h, &q);
  DistanceResult r;
  if (sd > 1e-12) {
    r.normal = (q - c) / (q - c).norm();
  } else {
    // Core inside (or on) the box: the outward normal of the nearest face points from B to A.
    const Vec3 local = b.pose.rotation().transpose() * (c - b.pose.translation());
    const Vec3 gap = local.cwiseAbs() - b.h;
    Index k = 0;
    gap.maxCoeff(&k);
    Vec3 face = Vec3::Zero();
    face(k) = local(k) >= 0.0 ? 1.0 : -1.0;
    r.normal = -(b.pose.rotation() * face);
  }
  r.distance = sd - a.radius;
  r.point_a = c + a.radius * r.normal;
  r.point_b = q;
  return r;
}

DistanceResult boxVsBox(const Core& a, const Core& b) {
  const GjkResult g = gjkDistance([&](const Vec3& d) { return coreSupport(a, d); },
                                  [&](const Vec3& d) { return coreSupport(b, d); },
                                  coreCenter(a) - coreCenter(b));
  DistanceResult r;
  if (!g.intersecting && g.distance > 1e-12) {
    r.distance = g.distance;
    r.point_a = g.point_a;
    r.point_b = g.point_b;
    r.normal = (g.point_b - g.point_a) / g.distance;
    return r;
  }
  // Overlapping: minimum penetration over the 15 separating-axis candidates.
  const Mat3& Ra = a.pose.rotation();
  const Mat3& Rb = b.pose.rotation();
  const Vec3 t = b.pose.translation() - a.pose.translation();
  std::vector<Vec3> axes;
  for (int i = 0; i < 3; ++i) {
    axes.emplace_back(Ra.col(i));
    axes.emplace_back(Rb.col(i));
  }
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      const Vec3 c = Ra.col(i).cross(Rb.col(j));
      if (c.norm() > 1e-9) {
        axes.emplace_back(c.normalized());
      }
    }
  }
  double best_overlap = std::numeric_limits<double>::infinity();
  Vec3 best_axis = Vec3::UnitZ();
  for (const Vec3& L : axes) {
    const double ra = a.h.dot((Ra.transpose() * L).cwiseAbs());
    const double rb = b.h.dot((Rb.transpose() * L).cwiseAbs());
    const double sep = L.dot(t);
    const double overlap = ra + rb - std::abs(sep);
    if (overlap < best_overlap) {
      best_overlap = overlap;
      best_axis = sep >= 0.0 ? L : Vec3(-L);
    }
  }
  r.distance = -std::max(0.0, best_overlap);
  r.normal = best_axis;
  r.point_a = coreSupport(a, best_axis);
  r.point_b = coreSupport(b, -best_axis);
  return r;
}

/// Non-plane core A against half-space B.
DistanceResult coreVsPlane(const Core& a, const Core& plane) {
  const Vec3 deepest = coreSupport(a, -plane.n) - a.radius * plane.n;
  const double dist = plane.n.dot(deepest) - plane.d;
  DistanceResult r;
  r.distance = dist;
  r.point_a = deepest;
  r.point_b = deepest - dist * plane.n;
  r.normal = -plane.n;
  return r;
}

}  // namespace

DistanceResult computeDistance(const ShapeGeometry& a, const SE3& T_a, const ShapeGeometry& b, const SE3& T_b) {
  const Core ca = makeCore(a, T_a);
  const Core cb = makeCore(b, T_b);
  using T = Core::Type;
  require(!(ca.type == T::Plane && cb.type == T::Plane), "computeDistance: half-space vs half-space is undefined");
  if (ca.type == T::Plane) {
    return flipped(coreVsPlane(cb, ca));
  }
  if (cb.type == T::Plane) {
    return coreVsPlane(ca, cb);
  }
  const bool a_box = ca.type == T::Box;
  const bool b_box = cb.type == T::Box;
  if (a_box && b_box) {
    return boxVsBox(ca, cb);
  }
  if (a_box) {
    return flipped(roundedVsBox(cb, ca));
  }
  if (b_box) {
    return roundedVsBox(ca, cb);
  }
  return roundedVsRounded(ca, cb);
}

DistanceResult computeDistance(const Shape& a, const SE3& T_world_parent_a, const Shape& b,
                               const SE3& T_world_parent_b) {
  return computeDistance(a.geometry, T_world_parent_a * a.pose, b.geometry, T_world_parent_b * b.pose);
}

bool shapesCollide(const ShapeGeometry& a, const SE3& T_a, const ShapeGeometry& b, const SE3& T_b, double margin) {
  return computeDistance(a, T_a, b, T_b).distance < margin;
}

}  // namespace mc
