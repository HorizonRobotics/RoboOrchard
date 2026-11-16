// The bounded TRF formulas below are a C++ port of SciPy 1.15.3
// scipy/optimize/_lsq/{trf,common}.py (SciPy BSD-3-Clause license).
// Dense exact/linear-loss/x_scale=1 configuration. See SCIPY_LICENSE.txt.
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "ik.hpp"

namespace {
constexpr double EPS = std::numeric_limits<double>::epsilon();
constexpr double INF = std::numeric_limits<double>::infinity();
Q feasible(Q x, const Q& lo, const Q& hi, double rstep) {
  for (int i = 0; i < x.size(); ++i) {
    double ld = x[i] - lo[i], ud = hi[i] - x[i];
    int active = 0;
    if (rstep == 0) {
      if (x[i] <= lo[i]) active = -1;
      if (x[i] >= hi[i]) active = 1;
    } else {
      if (std::isfinite(lo[i]) &&
          ld <= std::min(ud, rstep * std::max(1., std::abs(lo[i]))))
        active = -1;
      if (std::isfinite(hi[i]) &&
          ud <= std::min(ld, rstep * std::max(1., std::abs(hi[i]))))
        active = 1;
    }
    if (active == -1)
      x[i] = rstep == 0 ? std::nextafter(lo[i], hi[i])
                        : lo[i] + rstep * std::max(1., std::abs(lo[i]));
    if (active == 1)
      x[i] = rstep == 0 ? std::nextafter(hi[i], lo[i])
                        : hi[i] - rstep * std::max(1., std::abs(hi[i]));
    if (x[i] < lo[i] || x[i] > hi[i]) x[i] = .5 * (lo[i] + hi[i]);
  }
  return x;
}

void scaling(const Q& x, const Q& g, const Q& lo, const Q& hi, Q& v, Q& dv) {
  v.setOnes(x.size());
  dv.setZero(x.size());
  for (int i = 0; i < x.size(); ++i) {
    if (g[i] < 0 && std::isfinite(hi[i])) {
      v[i] = hi[i] - x[i];
      dv[i] = -1;
    }
    if (g[i] > 0 && std::isfinite(lo[i])) {
      v[i] = x[i] - lo[i];
      dv[i] = 1;
    }
  }
}
double bound_stride(const Q& x, const Q& p, const Q& lo, const Q& hi,
                    Eigen::VectorXi* hits = nullptr) {
  Q strides = Q::Constant(x.size(), INF);
  for (int i = 0; i < x.size(); ++i)
    if (p[i] != 0)
      strides[i] = std::max((lo[i] - x[i]) / p[i], (hi[i] - x[i]) / p[i]);
  double out = strides.minCoeff();
  if (hits)
    for (int i = 0; i < x.size(); ++i)
      (*hits)[i] = strides[i] == out ? (p[i] > 0 ? 1 : -1) : 0;
  return out;
}
double quadratic(const J& j, const Q& g, const Q& s, const Q& diag) {
  return .5 * ((j * s).squaredNorm() +
               (s.array().square() * diag.array()).sum()) +
         g.dot(s);
}
struct Quadratic {
  double a, b, c;
};
Quadratic build(const J& j, const Q& g, const Q& s, const Q& diag,
                const Q* s0 = nullptr) {
  F v = j * s;
  Quadratic q{
      .5 * (v.squaredNorm() + (s.array().square() * diag.array()).sum()),
      g.dot(s), 0};
  if (s0) {
    F u = j * (*s0);
    q.b += u.dot(v) + (s0->array() * diag.array() * s.array()).sum();
    q.c = .5 * u.squaredNorm() + g.dot(*s0) +
          .5 * (s0->array().square() * diag.array()).sum();
  }
  return q;
}
std::pair<double, double> minimize(Quadratic q, double lo, double hi) {
  double t = lo, y = lo * (q.a * lo + q.b) + q.c;
  auto test = [&](double n) {
    double z = n * (q.a * n + q.b) + q.c;
    if (z < y) {
      t = n;
      y = z;
    }
  };
  test(hi);
  if (q.a != 0) {
    double e = -.5 * q.b / q.a;
    if (lo < e && e < hi) test(e);
  }
  return {t, y};
}
struct Step {
  Q p, ph;
  double predicted;
};
Step select_step(const Q& x, const J& j, const Q& diag, const Q& g, Q p, Q ph,
                 const Q& d, double delta, const Q& lo, const Q& hi,
                 double theta) {
  if (((x + p).array() >= lo.array()).all() &&
      ((x + p).array() <= hi.array()).all())
    return {p, ph, -quadratic(j, g, ph, diag)};
  Eigen::VectorXi hits(x.size());
  double pstride = bound_stride(x, p, lo, hi, &hits);
  Q rh = ph;
  for (int i = 0; i < x.size(); ++i)
    if (hits[i]) rh[i] *= -1;
  Q r = d.array() * rh.array();
  p *= pstride;
  ph *= pstride;
  double a = rh.squaredNorm(), b = ph.dot(rh),
         c = ph.squaredNorm() - delta * delta;
  if (a == 0 || c > 0)
    throw std::runtime_error("TRF reflected intersection outside trust region");
  double root = std::sqrt(b * b - a * c), qq = -(b + std::copysign(root, b));
  double to_tr = std::max(qq / a, c / qq);
  double to_bound = bound_stride(x + p, r, lo, hi);
  double rstride = std::min(to_bound, to_tr), rl = 0, ru = -1;
  if (rstride > 0) {
    rl = (1 - theta) * pstride / rstride;
    ru = rstride == to_bound ? theta * to_bound : to_tr;
  }
  double rvalue = INF;
  if (rl <= ru) {
    auto opt = minimize(build(j, g, rh, diag, &ph), rl, ru);
    rh = rh * opt.first + ph;
    r = d.array() * rh.array();
    rvalue = opt.second;
  }
  p *= theta;
  ph *= theta;
  double pvalue = quadratic(j, g, ph, diag);
  Q ah = -g, ap = d.array() * ah.array();
  double atotr = delta / ah.norm(), atobound = bound_stride(x, ap, lo, hi);
  double astride = atobound < atotr ? theta * atobound : atotr;
  auto ao = minimize(build(j, g, ah, diag), 0, astride);
  ah *= ao.first;
  ap *= ao.first;
  if (pvalue < rvalue && pvalue < ao.second) return {p, ph, -pvalue};
  if (rvalue < pvalue && rvalue < ao.second) return {r, rh, -rvalue};
  return {ap, ah, -ao.second};
}
Q trust_region(const Q& uf, const Q& s, const J& v, double delta, double& alpha,
               int m) {
  Q suf = s.array() * uf.array();
  bool full = m >= s.size() && s[s.size() - 1] > EPS * m * s[0];
  if (full) {
    Q p = -v * (uf.array() / s.array()).matrix();
    if (p.norm() <= delta) {
      alpha = 0;
      return p;
    }
  }
  auto phi = [&](double al) {
    Q den = s.array().square() + al;
    double pn = (suf.array() / den.array()).matrix().norm();
    return std::pair<double, double>{
        pn - delta, -(suf.array().square() / den.array().cube()).sum() / pn};
  };
  double upper = suf.norm() / delta, lower = 0;
  if (full) {
    auto pd = phi(0);
    lower = -pd.first / pd.second;
  }
  if (!full && alpha == 0)
    alpha = std::max(.001 * upper, std::sqrt(lower * upper));
  for (int it = 0; it < 10; ++it) {
    if (alpha < lower || alpha > upper)
      alpha = std::max(.001 * upper, std::sqrt(lower * upper));
    auto pd = phi(alpha);
    if (pd.first < 0) upper = alpha;
    double ratio = pd.first / pd.second;
    lower = std::max(lower, alpha - ratio);
    alpha -= (pd.first + delta) * ratio / delta;
    if (std::abs(pd.first) < .01 * delta) break;
  }
  Q p = -v * (suf.array() / (s.array().square() + alpha)).matrix();
  p *= delta / p.norm();
  return p;
}
}  // namespace

Solver::Solver(const std::string& path, const std::string& base,
               const std::string& ee)
    : kin_(path, base, ee), lower_(kin_.lower()), upper_(kin_.upper()) {}

Q Solver::clamp(const Q& seed) const {
  if (seed.size() != lower_.size())
    throw std::invalid_argument("Seed length does not match active joints");
  Q q = seed;
  for (int i = 0; i < q.size(); ++i) {
    if (!std::isfinite(q[i])) q[i] = 0;
    q[i] = std::clamp(q[i], lower_[i], upper_[i]);
    if (std::isfinite(lower_[i]) && std::isfinite(upper_[i])) {
      if (q[i] <= lower_[i]) q[i] = std::nextafter(lower_[i], upper_[i]);
      if (q[i] >= upper_[i]) q[i] = std::nextafter(upper_[i], lower_[i]);
    }
  }
  return q;
}

double Solver::difference_step(const Q& q, int i) const {
  double h =
      std::sqrt(EPS) * (q[i] >= 0 ? 1 : -1) * std::max(1., std::abs(q[i]));
  double ld = q[i] - lower_[i], ud = upper_[i] - q[i];
  bool violated = q[i] + h < lower_[i] || q[i] + h > upper_[i];
  bool fitting = std::abs(h) <= std::max(ld, ud);
  if (violated && fitting) h *= -1;
  if (!fitting) h = ud >= ld ? ud : -ld;
  return h;
}

F Solver::residual(const Q& q, const Mat4& target, const Q& seed,
                   const Options& options) {
  Mat4 m = kin_.fk(q);
  int rows =
      options.orientation == 3 ? 12 : (options.orientation == -1 ? 3 : 6);
  F f(rows);
  f.head<3>() = m.topRightCorner<3, 1>() - target.topRightCorner<3, 1>();
  if (options.orientation == 3) {
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) f[3 + 3 * r + c] = m(r, c) - target(r, c);
  } else if (options.orientation >= 0) {
    f.tail<3>() = m.col(options.orientation).head<3>() -
                  target.col(options.orientation).head<3>();
  }
  // IKPy adds one scalar to every task residual, not a separate penalty block.
  if (options.regularization != 0)
    f.array() += options.regularization * (q - seed).norm();
  return f;
}

J Solver::residual_jacobian(const Q& q, const Q& seed, const Options& options) {
  J task = kin_.jacobian(q), j;
  if (options.orientation == 3)
    j = task;
  else {
    j.resize(options.orientation == -1 ? 3 : 6, q.size());
    j.topRows(3) = task.topRows(3);
    if (options.orientation >= 0)
      for (int r = 0; r < 3; ++r)
        j.row(3 + r) = task.row(3 + 3 * r + options.orientation);
  }
  if (options.regularization != 0) {
    double base = options.regularization * (q - seed).norm();
    // Preserve the one-sided norm derivative at the nondifferentiable seed.
    // Only this cheap scalar is differenced; FK/Jacobian reuse the same q.
    for (int i = 0; i < q.size(); ++i) {
      Q perturbed = q;
      perturbed[i] += difference_step(q, i);
      double dx = perturbed[i] - q[i];
      j.col(i).array() +=
          (options.regularization * (perturbed - seed).norm() - base) / dx;
    }
  }
  return j;
}

Result Solver::solve(const Mat4& target, const std::optional<Q>& raw_seed,
                     const Options& requested) {
  if (!target.allFinite()) throw std::invalid_argument("Target is non-finite");
  Options options = requested;
  if (options.orientation < -1 || options.orientation > 3)
    throw std::invalid_argument("Unknown orientation mode");
  if (!raw_seed) options.regularization = 0;
  const int n = lower_.size();
  const int max_nfev = options.max_nfev ? options.max_nfev : 100 * n;
  if (max_nfev < 1 || !std::isfinite(options.regularization))
    throw std::invalid_argument("Invalid solver options");
  const Q seed = clamp(raw_seed ? *raw_seed : Q::Zero(n));
  Q x = feasible(seed, lower_, upper_, 1e-10);
  F f = residual(x, target, seed, options);
  J j = residual_jacobian(x, seed, options);
  if (!f.allFinite())
    throw std::runtime_error("Initial residual is non-finite");
  double cost = .5 * f.squaredNorm();
  Q g = j.transpose() * f, v, dv;
  scaling(x, g, lower_, upper_, v, dv);
  double delta = (x.array() / v.array().sqrt()).matrix().norm();
  if (delta == 0) delta = 1;
  double alpha = 0;
  Result out;
  for (;;) {
    scaling(x, g, lower_, upper_, v, dv);
    out.optimality = (g.array() * v.array()).abs().maxCoeff();
    if (out.optimality < options.gtol) out.status = 1;
    if (out.status != 0 || out.nfev == max_nfev) break;
    Q d = v.array().sqrt(), diag = g.array() * dv.array();
    Q gh = d.array() * g.array();
    J jh = j * d.asDiagonal();
    J augmented = J::Zero(f.size() + n, n);
    augmented.topRows(f.size()) = jh;
    augmented.bottomRows(n) = diag.array().sqrt().matrix().asDiagonal();
    Eigen::JacobiSVD<J> svd(augmented,
                            Eigen::ComputeThinU | Eigen::ComputeThinV);
    Q uf = svd.matrixU().topRows(f.size()).transpose() * f;
    double theta = std::max(.995, 1 - out.optimality), actual = -1;
    double cost_new = cost;
    Q x_new = x;
    F f_new = f;
    while (actual <= 0 && out.nfev < max_nfev) {
      Q ph = trust_region(uf, svd.singularValues(), svd.matrixV(), delta, alpha,
                          f.size());
      Step step =
          select_step(x, jh, diag, gh, (d.array() * ph.array()).matrix(), ph, d,
                      delta, lower_, upper_, theta);
      x_new = feasible(x + step.p, lower_, upper_, 0);
      f_new = residual(x_new, target, seed, options);
      ++out.nfev;
      double hn = step.ph.norm();
      if (!f_new.allFinite()) {
        delta = .25 * hn;
        continue;
      }
      cost_new = .5 * f_new.squaredNorm();
      actual = cost - cost_new;
      double ratio = step.predicted > 0
                         ? actual / step.predicted
                         : (step.predicted == 0 && actual == 0 ? 1 : 0);
      double next_delta = delta;
      if (ratio < .25)
        next_delta = .25 * hn;
      else if (ratio > .75 && hn > .95 * delta)
        next_delta *= 2;
      bool ftol = actual < options.ftol * cost && ratio > .25;
      bool xtol = step.p.norm() < options.xtol * (options.xtol + x.norm());
      if (ftol || xtol) {
        out.status = ftol && xtol ? 4 : (ftol ? 2 : 3);
        break;
      }
      alpha *= delta / next_delta;
      delta = next_delta;
    }
    if (actual > 0) {
      x = x_new;
      f = f_new;
      cost = cost_new;
      j = residual_jacobian(x, seed, options);
      ++out.njev;
      g = j.transpose() * f;
    }
  }
  out.q = x;
  out.cost = cost;
  Mat4 m = kin_.fk(x);
  out.position_error =
      (m.topRightCorner<3, 1>() - target.topRightCorner<3, 1>()).norm();
  Eigen::Matrix3d relative =
      m.topLeftCorner<3, 3>().transpose() * target.topLeftCorner<3, 3>();
  Eigen::Quaterniond quat(relative);
  out.rotation_error = 2 * std::atan2(quat.vec().norm(), std::abs(quat.w()));
  out.accepted = !(out.position_error > options.position_tolerance ||
                   (options.orientation != -1 &&
                    out.rotation_error > options.orientation_tolerance));
  // Score the unmodified solution before optional clipping, as before.
  if (options.clip)
    for (int i = 0; i < n; ++i)
      out.q[i] = std::clamp(out.q[i], lower_[i], upper_[i]);
  return out;
}
