//! Bore contour with a parallel keyway (hub groove) per DIN 6885-1 / ISO 773.

use crate::profile::{self, Seg};
use std::f64::consts::PI;

/// (upper shaft diameter of the range, key width b, hub groove depth t2)
const DIN6885_1: &[(f64, f64, f64)] = &[
    (8.0, 2.0, 1.0),
    (10.0, 3.0, 1.4),
    (12.0, 4.0, 1.8),
    (17.0, 5.0, 2.3),
    (22.0, 6.0, 2.8),
    (30.0, 8.0, 3.3),
    (38.0, 10.0, 3.3),
    (44.0, 12.0, 3.3),
    (50.0, 14.0, 3.8),
    (58.0, 16.0, 4.3),
    (65.0, 18.0, 4.4),
    (75.0, 20.0, 4.9),
    (85.0, 22.0, 5.4),
    (95.0, 25.0, 5.4),
    (110.0, 28.0, 6.4),
    (130.0, 32.0, 7.4),
    (150.0, 36.0, 8.4),
    (170.0, 40.0, 9.4),
    (200.0, 45.0, 10.4),
];

/// Look up key width b and hub groove depth t2 for a bore/shaft diameter.
pub fn din6885(d: f64) -> Option<(f64, f64)> {
    if d < 6.0 {
        return None;
    }
    DIN6885_1
        .iter()
        .find(|(dmax, _, _)| d <= *dmax + 1e-9)
        .map(|&(_, b, t2)| (b, t2))
}

/// Shape of the bore. Everything is oriented towards +Y and then rotated by
/// the bore angle.
#[derive(Clone, Copy, Debug)]
pub enum Bore {
    Round,
    /// parallel keyway: width b, hub groove depth t2 (roof at d/2 + t2)
    Key(f64, f64),
    /// one flat, `across` = dimension from the flat to the opposite wall
    DFlat(f64),
    /// two parallel flats, `across` = dimension between them
    DoubleD(f64),
}

impl Bore {
    /// Signed offset of the flat from the bore axis, and the depth of material
    /// the flat adds compared with a plain round bore.
    pub fn flat(&self, d: f64) -> Option<(f64, f64)> {
        match *self {
            Bore::DFlat(a) => Some((a - d / 2.0, d - a)),
            Bore::DoubleD(a) => Some((a / 2.0, (d - a) / 2.0)),
            _ => None,
        }
    }
}

/// CCW bore contour in the transverse plane, as analytic segments.
pub fn bore_contour(d: f64, shape: Bore, angle: f64) -> Vec<Seg> {
    let r = d / 2.0;
    let o = [0.0, 0.0];
    let pt = |a: f64, rad: f64| [rad * (a + angle).cos(), rad * (a + angle).sin()];
    let xy = |x: f64, y: f64| {
        let (s, c) = angle.sin_cos();
        [c * x - s * y, s * x + c * y]
    };
    let arc = |a0: f64, a1: f64| Seg::Arc { c: o, r, a0: a0 + angle, a1: a1 + angle };
    let mut segs: Vec<Seg> = Vec::new();
    match shape {
        Bore::Round => segs.extend(profile::circle(o, r)),
        Bore::Key(b, t2) => {
            let hb = b / 2.0;
            let y1 = (r * r - hb * hb).max(0.0).sqrt();
            let th1 = y1.atan2(hb);
            segs.push(arc(PI - th1, th1 + 2.0 * PI)); // the long way round
            let (p0, p1, p2, p3) = (
                xy(hb, y1),
                xy(hb, r + t2),
                xy(-hb, r + t2),
                xy(-hb, y1),
            );
            segs.push(Seg::Line(p0, p1));
            segs.push(Seg::Line(p1, p2));
            segs.push(Seg::Line(p2, p3));
        }
        Bore::DFlat(_) => {
            let (h, _) = shape.flat(d).unwrap();
            let x1 = (r * r - h * h).max(0.0).sqrt();
            let th1 = h.atan2(x1);
            segs.push(arc(PI - th1, th1 + 2.0 * PI));
            segs.push(Seg::Line(pt(th1, r), pt(PI - th1, r))); // the flat
        }
        Bore::DoubleD(_) => {
            let (h, _) = shape.flat(d).unwrap();
            let x1 = (r * r - h * h).max(0.0).sqrt();
            let th1 = h.atan2(x1);
            segs.push(arc(-th1, th1));
            segs.push(Seg::Line(pt(th1, r), pt(PI - th1, r)));
            segs.push(arc(PI - th1, PI + th1));
            segs.push(Seg::Line(pt(PI + th1, r), pt(2.0 * PI - th1, r)));
        }
    }
    profile::tidy(segs)
}
