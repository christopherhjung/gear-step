//! Minimal B-spline machinery: basis functions, global interpolation
//! (Piegl & Tiller A9.1) and evaluation.

#[derive(Clone, Debug)]
pub struct Curve {
    pub deg: usize,
    pub ctrl: Vec<[f64; 3]>,
    /// distinct knot values
    pub knots: Vec<f64>,
    /// multiplicity of each distinct knot
    pub mult: Vec<usize>,
}

impl Curve {
    pub fn flat_knots(&self) -> Vec<f64> {
        let mut u = Vec::new();
        for (k, m) in self.knots.iter().zip(&self.mult) {
            for _ in 0..*m {
                u.push(*k);
            }
        }
        u
    }
    pub fn eval(&self, u: f64) -> [f64; 3] {
        let kv = self.flat_knots();
        let n = self.ctrl.len() - 1;
        let p = self.deg;
        let span = find_span(n, p, u, &kv);
        let b = basis_funs(span, u, p, &kv);
        let mut r = [0.0; 3];
        for j in 0..=p {
            for k in 0..3 {
                r[k] += b[j] * self.ctrl[span - p + j][k];
            }
        }
        r
    }
    /// Rotate the control net about the Z axis and lift it to `z`.
    pub fn transformed(&self, ang: f64, z: f64) -> Curve {
        let (s, c) = ang.sin_cos();
        Curve {
            deg: self.deg,
            ctrl: self
                .ctrl
                .iter()
                .map(|p| [c * p[0] - s * p[1], s * p[0] + c * p[1], z])
                .collect(),
            knots: self.knots.clone(),
            mult: self.mult.clone(),
        }
    }
}

pub fn find_span(n: usize, p: usize, u: f64, kv: &[f64]) -> usize {
    if u >= kv[n + 1] {
        return n;
    }
    if u <= kv[p] {
        return p;
    }
    let (mut lo, mut hi) = (p, n + 1);
    let mut mid = (lo + hi) / 2;
    while u < kv[mid] || u >= kv[mid + 1] {
        if u < kv[mid] {
            hi = mid
        } else {
            lo = mid
        }
        mid = (lo + hi) / 2;
    }
    mid
}

pub fn basis_funs(i: usize, u: f64, p: usize, kv: &[f64]) -> Vec<f64> {
    let mut n = vec![0.0; p + 1];
    let mut left = vec![0.0; p + 1];
    let mut right = vec![0.0; p + 1];
    n[0] = 1.0;
    for j in 1..=p {
        left[j] = u - kv[i + 1 - j];
        right[j] = kv[i + j] - u;
        let mut saved = 0.0;
        for r in 0..j {
            let den = right[r + 1] + left[j - r];
            let temp = if den.abs() < 1e-300 { 0.0 } else { n[r] / den };
            n[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        n[j] = saved;
    }
    n
}

/// Chord length parametrisation, normalised to [0, 1].
pub fn chord_params(pts: &[[f64; 3]]) -> Vec<f64> {
    let n = pts.len();
    let mut d = vec![0.0; n];
    for i in 1..n {
        let (a, b) = (pts[i - 1], pts[i]);
        d[i] = d[i - 1]
            + ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt();
    }
    let tot = d[n - 1];
    if tot <= 0.0 {
        return (0..n).map(|i| i as f64 / (n - 1) as f64).collect();
    }
    d.iter().map(|x| x / tot).collect()
}

pub fn uniform_params(n: usize) -> Vec<f64> {
    (0..n).map(|i| i as f64 / (n - 1) as f64).collect()
}

/// Clamped knot vector by knot averaging.
fn averaged_knots(params: &[f64], p: usize) -> Vec<f64> {
    let n = params.len() - 1;
    let mut kv = vec![0.0; n + p + 2];
    for i in 0..=p {
        kv[i] = 0.0;
        kv[n + 1 + i] = 1.0;
    }
    for j in 1..=(n.saturating_sub(p)) {
        let s: f64 = params[j..j + p].iter().sum();
        kv[j + p] = s / p as f64;
    }
    kv
}

fn solve(mut a: Vec<Vec<f64>>, mut b: Vec<[f64; 3]>) -> Vec<[f64; 3]> {
    let n = a.len();
    for k in 0..n {
        let mut piv = k;
        for i in k + 1..n {
            if a[i][k].abs() > a[piv][k].abs() {
                piv = i;
            }
        }
        a.swap(k, piv);
        b.swap(k, piv);
        let d = a[k][k];
        if d.abs() < 1e-300 {
            continue;
        }
        for i in k + 1..n {
            let f = a[i][k] / d;
            if f == 0.0 {
                continue;
            }
            for j in k..n {
                a[i][j] -= f * a[k][j];
            }
            for j in 0..3 {
                b[i][j] -= f * b[k][j];
            }
        }
    }
    let mut x = vec![[0.0; 3]; n];
    for k in (0..n).rev() {
        let mut s = b[k];
        for j in k + 1..n {
            for c in 0..3 {
                s[c] -= a[k][j] * x[j][c];
            }
        }
        let d = a[k][k];
        for c in 0..3 {
            x[k][c] = if d.abs() < 1e-300 { s[c] } else { s[c] / d };
        }
    }
    x
}

/// Global interpolation: the curve passes through every point.
pub fn interpolate(pts: &[[f64; 3]], params: &[f64], deg: usize) -> Curve {
    let n = pts.len() - 1;
    let p = deg.min(n);
    let kv = averaged_knots(params, p);
    let mut a = vec![vec![0.0; n + 1]; n + 1];
    for (k, &u) in params.iter().enumerate() {
        let span = find_span(n, p, u, &kv);
        let b = basis_funs(span, u, p, &kv);
        for j in 0..=p {
            a[k][span - p + j] = b[j];
        }
    }
    let ctrl = solve(a, pts.to_vec());
    let (knots, mult) = compress(&kv);
    Curve { deg: p, ctrl, knots, mult }
}

pub fn compress(kv: &[f64]) -> (Vec<f64>, Vec<usize>) {
    let mut k = Vec::new();
    let mut m = Vec::new();
    for &v in kv {
        if let Some(last) = k.last() {
            if (v - *last as f64).abs() < 1e-12 {
                *m.last_mut().unwrap() += 1;
                continue;
            }
        }
        k.push(v);
        m.push(1);
    }
    (k, m)
}
