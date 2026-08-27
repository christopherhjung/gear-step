//! Triangulation of the same swept solid the STEP writer exports, for display.
//! Cylinders and splines become polygons here - this is the display mesh, not
//! the model; the STEP file stays analytic.

use crate::brep::Model;
use crate::earcut::earcut;
use crate::profile::Seg;
use std::f64::consts::PI;

pub struct Mesh {
    /// xyz triplets
    pub pos: Vec<f32>,
    /// one normal per position
    pub nrm: Vec<f32>,
    pub idx: Vec<u32>,
    /// hard edges as xyz pairs, for the wireframe overlay
    pub lines: Vec<f32>,
}

impl Mesh {
    pub fn tris(&self) -> usize {
        self.idx.len() / 3
    }
}

/// Cosine of the largest angle between two segment normals still treated as a
/// smooth junction.
const SMOOTH: f64 = 0.9063; // 25 degrees
/// Chordal resolution of arcs, in radians.
const ARC_STEP: f64 = 0.035; // ~2 degrees

fn unit(v: [f64; 2]) -> [f64; 2] {
    let l = v[0].hypot(v[1]).max(1e-15);
    [v[0] / l, v[1] / l]
}

fn rot(p: [f64; 2], a: f64) -> [f64; 2] {
    let (s, c) = a.sin_cos();
    [c * p[0] - s * p[1], s * p[0] + c * p[1]]
}

/// Sample one segment as (point, outward normal) pairs, both ends included.
fn sample_seg(s: &Seg, sign: f64) -> Vec<([f64; 2], [f64; 2])> {
    let nrm = |t: [f64; 2]| [sign * t[1], -sign * t[0]];
    match s {
        Seg::Line(a, b) => {
            let t = unit([b[0] - a[0], b[1] - a[1]]);
            vec![(*a, nrm(t)), (*b, nrm(t))]
        }
        Seg::Arc { c, r, a0, a1 } => {
            let sweep = a1 - a0;
            let k = ((sweep.abs() / ARC_STEP).ceil() as usize).max(1);
            (0..=k)
                .map(|i| {
                    let a = a0 + sweep * i as f64 / k as f64;
                    let t = unit([-sweep.signum() * a.sin(), sweep.signum() * a.cos()]);
                    ([c[0] + r * a.cos(), c[1] + r * a.sin()], nrm(t))
                })
                .collect()
        }
        Seg::Curve(p) => {
            let n = p.len();
            (0..n)
                .map(|i| {
                    let (a, b) = if i == 0 {
                        (p[0], p[1])
                    } else if i == n - 1 {
                        (p[n - 2], p[n - 1])
                    } else {
                        (p[i - 1], p[i + 1])
                    };
                    (p[i], nrm(unit([b[0] - a[0], b[1] - a[1]])))
                })
                .collect()
        }
    }
}

/// One entry per column of the lateral band: a point on the contour, its
/// outward normal, and whether a hard edge starts here.
struct Column {
    p: [f64; 2],
    n: [f64; 2],
    hard: bool,
}

fn columns(segs: &[Seg], sign: f64) -> Vec<Column> {
    let per: Vec<Vec<([f64; 2], [f64; 2])>> =
        segs.iter().map(|s| sample_seg(s, sign)).collect();
    let m = per.len();
    let mut out: Vec<Column> = Vec::new();
    for i in 0..m {
        let cur = &per[i];
        let prev = &per[(i + m - 1) % m];
        let (pp, pn) = *prev.last().unwrap();
        let (cp, cn) = cur[0];
        let dot = pn[0] * cn[0] + pn[1] * cn[1];
        if dot > SMOOTH {
            out.push(Column {
                p: cp,
                n: unit([pn[0] + cn[0], pn[1] + cn[1]]),
                hard: false,
            });
        } else {
            out.push(Column { p: pp, n: pn, hard: true });
            out.push(Column { p: cp, n: cn, hard: true });
        }
        for c in &cur[1..cur.len() - 1] {
            out.push(Column { p: c.0, n: c.1, hard: false });
        }
    }
    out
}

/// Closed polyline of a contour with consecutive duplicates removed.
pub fn sample_contour(segs: &[Seg]) -> Vec<[f64; 2]> {
    let mut v: Vec<[f64; 2]> = Vec::new();
    for s in segs {
        for (q, _) in sample_seg(s, 1.0) {
            if v
                .last()
                .map(|l: &[f64; 2]| (l[0] - q[0]).hypot(l[1] - q[1]) > 1e-9)
                .unwrap_or(true)
            {
                v.push(q);
            }
        }
    }
    while v.len() > 1 {
        let (a, b) = (v[0], *v.last().unwrap());
        if (a[0] - b[0]).hypot(a[1] - b[1]) < 1e-9 {
            v.pop();
        } else {
            break;
        }
    }
    v
}

struct Build {
    pos: Vec<f32>,
    nrm: Vec<f32>,
    idx: Vec<u32>,
    lines: Vec<f32>,
}

impl Build {
    fn vertex(&mut self, p: [f64; 3], n: [f64; 3]) -> u32 {
        let id = (self.pos.len() / 3) as u32;
        self.pos.extend_from_slice(&[p[0] as f32, p[1] as f32, p[2] as f32]);
        let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt().max(1e-15);
        self.nrm
            .extend_from_slice(&[(n[0] / l) as f32, (n[1] / l) as f32, (n[2] / l) as f32]);
        id
    }
    fn tri(&mut self, a: u32, b: u32, c: u32) {
        self.idx.extend_from_slice(&[a, b, c]);
    }
    fn line(&mut self, a: [f64; 3], b: [f64; 3]) {
        self.lines.extend_from_slice(&[
            a[0] as f32, a[1] as f32, a[2] as f32, b[0] as f32, b[1] as f32, b[2] as f32,
        ]);
    }
}

/// Number of axial sections used for the display mesh: enough that the twist
/// stays smooth.
fn display_layers(twist: f64) -> usize {
    if twist.abs() < 1e-12 {
        2
    } else {
        ((twist.abs() / (3.0 * PI / 180.0)).ceil() as usize + 1).clamp(2, 200)
    }
}

pub fn tessellate(model: &Model) -> Mesh {
    let mut b = Build { pos: Vec::new(), nrm: Vec::new(), idx: Vec::new(), lines: Vec::new() };
    let dz = model.z1 - model.z0;
    let nlay = display_layers(model.twist);

    let rings: Vec<(&Vec<Seg>, f64)> = std::iter::once((&model.outer, 1.0))
        .chain(model.holes.iter().map(|h| (h, -1.0)))
        .collect();

    // ---- lateral bands -------------------------------------------------
    for (ci, (segs, sign)) in rings.iter().enumerate() {
        let twist = if ci == 0 { model.twist } else { 0.0 };
        let layers = if ci == 0 { nlay } else { 2 };
        let cols = columns(segs, *sign);
        let nc = cols.len();
        if nc < 3 {
            continue;
        }
        let base = (b.pos.len() / 3) as u32;
        for l in 0..layers {
            let v = l as f64 / (layers - 1) as f64;
            let ang = twist * v;
            let z = model.z0 + dz * v;
            for c in &cols {
                let q = rot(c.p, ang);
                let n2 = rot(c.n, ang);
                // t = travel tangent; for the outer ring n = (t.y, -t.x)
                let t = [-sign * c.n[1], sign * c.n[0]];
                let nz = sign * twist * (t[0] * c.p[0] + t[1] * c.p[1]);
                b.vertex([q[0], q[1], z], [n2[0] * dz, n2[1] * dz, nz]);
            }
        }
        let at = |l: usize, j: usize| base + (l * nc + j % nc) as u32;
        for l in 0..layers - 1 {
            for j in 0..nc {
                let (p0, p1) = (cols[j].p, cols[(j + 1) % nc].p);
                if (p0[0] - p1[0]).hypot(p0[1] - p1[1]) < 1e-12 {
                    continue; // the zero width quad of a hard edge
                }
                let (a, bb, c, d) = (at(l, j), at(l, j + 1), at(l + 1, j + 1), at(l + 1, j));
                if *sign > 0.0 {
                    b.tri(a, bb, c);
                    b.tri(a, c, d);
                } else {
                    b.tri(a, c, bb);
                    b.tri(a, d, c);
                }
            }
        }
        // hard edges: the two end profiles and the vertical creases
        for l in [0usize, layers - 1] {
            let v = l as f64 / (layers - 1) as f64;
            let ang = twist * v;
            let z = model.z0 + dz * v;
            for j in 0..nc {
                let p0 = rot(cols[j].p, ang);
                let p1 = rot(cols[(j + 1) % nc].p, ang);
                b.line([p0[0], p0[1], z], [p1[0], p1[1], z]);
            }
        }
        for (j, c) in cols.iter().enumerate() {
            if !c.hard {
                continue;
            }
            // one crease per pair of coincident columns
            let nx = &cols[(j + 1) % nc];
            if (c.p[0] - nx.p[0]).hypot(c.p[1] - nx.p[1]) < 1e-12 {
                continue;
            }
            let mut prev: Option<[f64; 3]> = None;
            for l in 0..layers {
                let v = l as f64 / (layers - 1) as f64;
                let q = rot(c.p, twist * v);
                let cur = [q[0], q[1], model.z0 + dz * v];
                if let Some(p) = prev {
                    b.line(p, cur);
                }
                prev = Some(cur);
            }
        }
    }

    // ---- end caps ------------------------------------------------------
    let outer_pts = sample_contour(&model.outer);
    let hole_pts: Vec<Vec<[f64; 2]>> = model.holes.iter().map(|h| sample_contour(h)).collect();
    for (top, z) in [(false, model.z0), (true, model.z1)] {
        let ang = if top { model.twist } else { 0.0 };
        let mut flat: Vec<f64> = Vec::new();
        for p in &outer_pts {
            let q = rot(*p, ang);
            flat.push(q[0]);
            flat.push(q[1]);
        }
        let mut hi: Vec<usize> = Vec::new();
        for h in &hole_pts {
            hi.push(flat.len() / 2);
            for p in h {
                flat.push(p[0]);
                flat.push(p[1]);
            }
        }
        let tris = earcut(&flat, &hi);
        let nz = if top { 1.0 } else { -1.0 };
        let base = (b.pos.len() / 3) as u32;
        for i in 0..flat.len() / 2 {
            b.vertex([flat[2 * i], flat[2 * i + 1], z], [0.0, 0.0, nz]);
        }
        for t in tris.chunks(3) {
            if t.len() < 3 {
                break;
            }
            if top {
                b.tri(base + t[0] as u32, base + t[1] as u32, base + t[2] as u32);
            } else {
                b.tri(base + t[0] as u32, base + t[2] as u32, base + t[1] as u32);
            }
        }
    }

    Mesh { pos: b.pos, nrm: b.nrm, idx: b.idx, lines: b.lines }
}
