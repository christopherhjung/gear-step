//! Turns stacked 2D contours into a closed, manifold, planar-faced B-rep.

use std::collections::HashMap;

pub struct Brep {
    pub verts: Vec<[f64; 3]>,
    /// per face: outer loop first, then inner loops, already oriented so that
    /// the Newell normal of the outer loop points out of the solid
    pub faces: Vec<Vec<Vec<usize>>>,
}

pub fn newell(verts: &[[f64; 3]], loop_: &[usize]) -> [f64; 3] {
    let mut n = [0.0f64; 3];
    for i in 0..loop_.len() {
        let a = verts[loop_[i]];
        let b = verts[loop_[(i + 1) % loop_.len()]];
        n[0] += (a[1] - b[1]) * (a[2] + b[2]);
        n[1] += (a[2] - b[2]) * (a[0] + b[0]);
        n[2] += (a[0] - b[0]) * (a[1] + b[1]);
    }
    let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
    if l < 1e-14 {
        [0.0, 0.0, 0.0]
    } else {
        [n[0] / l, n[1] / l, n[2] / l]
    }
}

/// `outer`: one contour per axial layer (all the same length, CCW seen from +Z).
/// `holes`: straight through holes (CCW seen from +Z).
/// `zs`: layer heights, same length as `outer`.
pub fn build(
    outer: &[Vec<[f64; 2]>],
    holes: &[Vec<[f64; 2]>],
    zs: &[f64],
    triangulate: bool,
) -> Brep {
    let nl = outer.len();
    let np = outer[0].len();
    let mut verts: Vec<[f64; 3]> = Vec::with_capacity(nl * np);
    for (li, layer) in outer.iter().enumerate() {
        for p in layer {
            verts.push([p[0], p[1], zs[li]]);
        }
    }
    let ov = |layer: usize, i: usize| layer * np + i % np;

    let mut hole_base = Vec::new();
    for h in holes {
        hole_base.push(verts.len());
        for p in h {
            verts.push([p[0], p[1], zs[0]]);
        }
        for p in h {
            verts.push([p[0], p[1], *zs.last().unwrap()]);
        }
    }

    let mut faces: Vec<Vec<Vec<usize>>> = Vec::new();

    // ---- bottom (normal -Z) and top (normal +Z) ----
    let mut bottom: Vec<Vec<usize>> = vec![(0..np).rev().map(|i| ov(0, i)).collect()];
    let mut top: Vec<Vec<usize>> = vec![(0..np).map(|i| ov(nl - 1, i)).collect()];
    for (hi, h) in holes.iter().enumerate() {
        let n = h.len();
        let b0 = hole_base[hi];
        bottom.push((0..n).map(|i| b0 + i).collect());
        top.push((0..n).rev().map(|i| b0 + n + i).collect());
    }
    faces.push(bottom);
    faces.push(top);

    // ---- outer lateral band ----
    for l in 0..nl - 1 {
        for i in 0..np {
            let (a, b, c, d) = (ov(l, i), ov(l, i + 1), ov(l + 1, i + 1), ov(l + 1, i));
            if triangulate {
                faces.push(vec![vec![a, b, c]]);
                faces.push(vec![vec![a, c, d]]);
            } else {
                faces.push(vec![vec![a, b, c, d]]);
            }
        }
    }

    // ---- hole walls (normal points into the hole) ----
    for (hi, h) in holes.iter().enumerate() {
        let n = h.len();
        let b0 = hole_base[hi];
        for i in 0..n {
            let j = (i + 1) % n;
            faces.push(vec![vec![b0 + j, b0 + i, b0 + n + i, b0 + n + j]]);
        }
    }

    // drop degenerate faces
    faces.retain(|f| {
        let nrm = newell(&verts, &f[0]);
        nrm[0].abs() + nrm[1].abs() + nrm[2].abs() > 1e-12
    });

    Brep { verts, faces }
}

impl Brep {
    /// Every edge must be used exactly twice, once in each direction, and the
    /// Euler characteristic must be 2 - 2*genus.
    pub fn validate(&self, genus: i64) -> Result<String, String> {
        let mut use_count: HashMap<(usize, usize), (u32, u32)> = HashMap::new();
        for f in &self.faces {
            for lp in f {
                for i in 0..lp.len() {
                    let (a, b) = (lp[i], lp[(i + 1) % lp.len()]);
                    if a == b {
                        return Err("zero length edge".into());
                    }
                    let key = (a.min(b), a.max(b));
                    let e = use_count.entry(key).or_insert((0, 0));
                    if a < b {
                        e.0 += 1
                    } else {
                        e.1 += 1
                    }
                }
            }
        }
        let bad = use_count.values().filter(|&&(f, r)| f != 1 || r != 1).count();
        if bad > 0 {
            return Err(format!("{} edges are not shared by exactly two opposite half edges", bad));
        }
        let v = self.verts.len() as i64;
        let e = use_count.len() as i64;
        let f = self.faces.len() as i64;
        let rings: i64 = self.faces.iter().map(|f| f.len() as i64 - 1).sum();
        // Euler-Poincare:  V - E + F - R = 2*(S - G)
        let chi = v - e + f - rings;
        let want = 2 - 2 * genus;
        if chi != want {
            return Err(format!("Euler characteristic {} != {} (expected genus {})", chi, want, genus));
        }
        Ok(format!(
            "closed manifold shell: V={} E={} F={} R={} chi={}",
            v, e, f, rings, chi
        ))
    }
}
