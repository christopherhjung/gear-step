//! The transverse profile as analytic segments instead of a polyline.

use std::f64::consts::PI;

#[derive(Clone, Debug)]
pub enum Seg {
    Line([f64; 2], [f64; 2]),
    /// arc travelled from a0 to a1 (CCW when a1 > a0)
    Arc {
        c: [f64; 2],
        r: f64,
        a0: f64,
        a1: f64,
    },
    /// interpolation points of a spline flank, in travel order
    Curve(Vec<[f64; 2]>),
}

impl Seg {
    pub fn start(&self) -> [f64; 2] {
        match self {
            Seg::Line(a, _) => *a,
            Seg::Arc { c, r, a0, .. } => [c[0] + r * a0.cos(), c[1] + r * a0.sin()],
            Seg::Curve(p) => p[0],
        }
    }
    pub fn end(&self) -> [f64; 2] {
        match self {
            Seg::Line(_, b) => *b,
            Seg::Arc { c, r, a1, .. } => [c[0] + r * a1.cos(), c[1] + r * a1.sin()],
            Seg::Curve(p) => *p.last().unwrap(),
        }
    }
    pub fn is_degenerate(&self) -> bool {
        match self {
            Seg::Arc { r, a0, a1, .. } => (a1 - a0).abs() * r < 1e-9,
            _ => {
                let (a, b) = (self.start(), self.end());
                (a[0] - b[0]).hypot(a[1] - b[1]) < 1e-9
            }
        }
    }
    /// Polyline approximation, used for the SVG dump and for checks.
    pub fn sample(&self, n: usize) -> Vec<[f64; 2]> {
        match self {
            Seg::Line(a, b) => vec![*a, *b],
            Seg::Arc { c, r, a0, a1 } => {
                let k = ((n as f64 * (a1 - a0).abs() / (2.0 * PI)).ceil() as usize).max(2);
                (0..=k)
                    .map(|i| {
                        let a = a0 + (a1 - a0) * i as f64 / k as f64;
                        [c[0] + r * a.cos(), c[1] + r * a.sin()]
                    })
                    .collect()
            }
            Seg::Curve(p) => p.clone(),
        }
    }
    /// Unit tangent at the midpoint, in travel direction.
    pub fn mid_tangent(&self) -> ([f64; 2], [f64; 2]) {
        match self {
            Seg::Line(a, b) => {
                let d = [b[0] - a[0], b[1] - a[1]];
                let l = d[0].hypot(d[1]).max(1e-15);
                ([(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0], [d[0] / l, d[1] / l])
            }
            Seg::Arc { c, r, a0, a1 } => {
                let am = 0.5 * (a0 + a1);
                let s = (a1 - a0).signum();
                (
                    [c[0] + r * am.cos(), c[1] + r * am.sin()],
                    [-s * am.sin(), s * am.cos()],
                )
            }
            Seg::Curve(p) => {
                let i = p.len() / 2;
                let (a, b) = (p[i - 1], p[i]);
                let d = [b[0] - a[0], b[1] - a[1]];
                let l = d[0].hypot(d[1]).max(1e-15);
                ([(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0], [d[0] / l, d[1] / l])
            }
        }
    }
}

/// Split arcs into pieces of at most 90 degrees; drop degenerate segments.
pub fn tidy(segs: Vec<Seg>) -> Vec<Seg> {
    let mut out = Vec::new();
    for s in segs {
        if s.is_degenerate() {
            continue;
        }
        match s {
            Seg::Arc { c, r, a0, a1 } => {
                let k = ((a1 - a0).abs() / (PI / 2.0)).ceil().max(1.0) as usize;
                for i in 0..k {
                    out.push(Seg::Arc {
                        c,
                        r,
                        a0: a0 + (a1 - a0) * i as f64 / k as f64,
                        a1: a0 + (a1 - a0) * (i + 1) as f64 / k as f64,
                    });
                }
            }
            other => out.push(other),
        }
    }
    out
}

/// Closed CCW circle as four quadrant arcs.
pub fn circle(c: [f64; 2], r: f64) -> Vec<Seg> {
    (0..4)
        .map(|i| Seg::Arc {
            c,
            r,
            a0: i as f64 * PI / 2.0,
            a1: (i + 1) as f64 * PI / 2.0,
        })
        .collect()
}

/// Check that a contour is closed and continuous.
pub fn check_closed(segs: &[Seg]) -> Result<(), String> {
    for i in 0..segs.len() {
        let a = segs[i].end();
        let b = segs[(i + 1) % segs.len()].start();
        let d = (a[0] - b[0]).hypot(a[1] - b[1]);
        if d > 1e-7 {
            return Err(format!("contour gap of {:.3e} mm at segment {}", d, i));
        }
    }
    Ok(())
}
