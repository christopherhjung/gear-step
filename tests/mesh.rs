//! The display mesh has to describe the same solid as the STEP file: closed,
//! consistently wound, and of the right volume.

use gear_step::api::{self, Key, Spec};
use std::collections::HashMap;

fn quant(v: f32) -> i64 {
    (v as f64 * 1e5).round() as i64
}

/// (closed?, worst |signed area| of a degenerate triangle, volume)
fn check(mesh: &gear_step::mesh::Mesh) -> (Result<String, String>, f64) {
    let p = |i: u32| {
        let i = i as usize * 3;
        [mesh.pos[i] as f64, mesh.pos[i + 1] as f64, mesh.pos[i + 2] as f64]
    };
    let key = |i: u32| {
        let i = i as usize * 3;
        (quant(mesh.pos[i]), quant(mesh.pos[i + 1]), quant(mesh.pos[i + 2]))
    };
    let mut edges: HashMap<((i64, i64, i64), (i64, i64, i64)), i32> = HashMap::new();
    let mut vol = 0.0;
    let mut degenerate = 0;
    for t in mesh.idx.chunks(3) {
        let (a, b, c) = (p(t[0]), p(t[1]), p(t[2]));
        let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let n = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ];
        if (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt() < 1e-14 {
            degenerate += 1;
        }
        vol += (a[0] * n[0] + a[1] * n[1] + a[2] * n[2]) / 6.0;
        for (x, y) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            let (kx, ky) = (key(x), key(y));
            if kx == ky {
                continue;
            }
            let (lo, hi, dir) = if kx < ky { (kx, ky, 1) } else { (ky, kx, -1) };
            *edges.entry((lo, hi)).or_insert(0) += dir;
        }
    }
    let bad = edges.values().filter(|&&v| v != 0).count();
    let res = if degenerate > 0 {
        Err(format!("{} degenerate triangles", degenerate))
    } else if bad > 0 {
        Err(format!("{} of {} edges are not shared by two opposite triangles", bad, edges.len()))
    } else {
        Ok(format!("closed: {} triangles, {} edges", mesh.idx.len() / 3, edges.len()))
    };
    (res, vol)
}

/// Every stored vertex normal has to agree with the faces that use it. The
/// triangles are wound outward (the volume check proves it), so a vertex normal
/// that disagrees with its own faces is simply wrong.
fn normal_check(mesh: &gear_step::mesh::Mesh) -> (f64, usize, usize) {
    let pos = |i: u32| {
        let i = i as usize * 3;
        [mesh.pos[i] as f64, mesh.pos[i + 1] as f64, mesh.pos[i + 2] as f64]
    };
    let nrm = |i: u32| {
        let i = i as usize * 3;
        [mesh.nrm[i] as f64, mesh.nrm[i + 1] as f64, mesh.nrm[i + 2] as f64]
    };
    let mut worst: f64 = 1.0;
    let mut bad = 0;
    let mut flipped = 0;
    for t in mesh.idx.chunks(3) {
        let (a, b, c) = (pos(t[0]), pos(t[1]), pos(t[2]));
        let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
        let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
        let mut f = [
            u[1] * v[2] - u[2] * v[1],
            u[2] * v[0] - u[0] * v[2],
            u[0] * v[1] - u[1] * v[0],
        ];
        let l = (f[0] * f[0] + f[1] * f[1] + f[2] * f[2]).sqrt();
        if l < 1e-14 {
            continue;
        }
        for k in 0..3 {
            f[k] /= l;
        }
        for &i in t {
            let n = nrm(i);
            let d = n[0] * f[0] + n[1] * f[1] + n[2] * f[2];
            worst = worst.min(d);
            if d < 0.9 {
                bad += 1;
            }
            if d < 0.0 {
                flipped += 1;
            }
        }
    }
    (worst, bad, flipped)
}

/// The normal of an involute is tangent to the base circle: for a point P on
/// the flank with unit normal n, |P x n| == r_b exactly. That is an absolute
/// check on the mesh normals, independent of how finely the flank is chopped.
/// Returns the worst error in mm and the angle it corresponds to, in degrees.
fn involute_normal_error(b: &gear_step::api::Built, mesh: &gear_step::mesh::Mesh) -> (f64, f64) {
    let (r_b, r_form, r_a) = (b.g.r_b, b.g.r_form, b.g.r_a);
    let mut worst = 0.0f64;
    let mut worst_deg = 0.0f64;
    for i in 0..mesh.pos.len() / 3 {
        let (x, y, z) = (
            mesh.pos[i * 3] as f64,
            mesh.pos[i * 3 + 1] as f64,
            mesh.pos[i * 3 + 2] as f64,
        );
        if z.abs() > 1e-9 {
            continue; // the bottom layer only: there the section is unrotated
        }
        let (nx, ny) = (mesh.nrm[i * 3] as f64, mesh.nrm[i * 3 + 1] as f64);
        let l = nx.hypot(ny);
        if l < 0.5 {
            continue; // an end cap vertex, normal along the axis
        }
        let r = x.hypot(y);
        // strictly between the form and the tip circle is the involute flank,
        // clear of the root fillet, the root cylinder and the tip land
        if r <= r_form * 1.002 || r >= r_a * 0.998 {
            continue;
        }
        let arm = (x * ny / l - y * nx / l).abs();
        let e = (arm - r_b).abs();
        if e > worst {
            worst = e;
            worst_deg = (e / (r * r - r_b * r_b).max(1e-12).sqrt()).atan().to_degrees();
        }
    }
    (worst, worst_deg)
}

/// Cross-section area from the sampled contours (shoelace), outer minus holes.
fn section_area(b: &gear_step::api::Built) -> f64 {
    let shoelace = |c: &Vec<[f64; 2]>| {
        let n = c.len();
        let mut s = 0.0;
        for i in 0..n {
            let (a, b) = (c[i], c[(i + 1) % n]);
            s += a[0] * b[1] - b[0] * a[1];
        }
        s / 2.0
    };
    let cs = b.section();
    shoelace(&cs[0]) - cs[1..].iter().map(shoelace).sum::<f64>()
}

fn case(name: &str, spec: Spec) {
    let b = api::build(&spec).unwrap_or_else(|e| panic!("{}: {}", name, e));
    let mesh = b.mesh();
    let (res, vol) = check(&mesh);
    let want = section_area(&b) * b.spec.width;
    let err = (vol - want).abs() / want;
    let (worst, bad, flipped) = normal_check(&mesh);
    let (inv_e, inv_deg) = involute_normal_error(&b, &mesh);
    println!(
        "{:<24} {:>7} tris  vol {:.0e}  dot {:.4} ({} off, {} flipped)  flank normal {:>7.4} deg  {}",
        name,
        mesh.idx.len() / 3,
        err,
        worst,
        bad,
        flipped,
        inv_deg,
        match &res {
            Ok(_) => "closed",
            Err(_) => "OPEN",
        }
    );
    let _ = inv_e;
    res.unwrap_or_else(|e| panic!("{}: {}", name, e));
    assert!(err < 2e-3, "{}: volume off by {:.3}%", name, err * 100.0);
    assert_eq!(flipped, 0, "{}: {} vertex normals point into the solid", name, flipped);
    assert!(
        worst > 0.9,
        "{}: a vertex normal is {:.1} deg off the face that uses it",
        name,
        worst.acos().to_degrees()
    );
    assert!(
        inv_deg < 0.5,
        "{}: a flank normal misses the exact involute by {:.3} deg",
        name,
        inv_deg
    );
}

#[test]
fn meshes_are_closed_solids() {
    case("plain spur", Spec { z: 24, m_n: 2.0, width: 12.0, ..Spec::default() });
    case(
        "keyway bore",
        Spec { z: 24, m_n: 2.0, width: 12.0, bore: 20.0, ..Spec::default() },
    );
    case(
        "round bore",
        Spec { z: 24, m_n: 2.0, width: 12.0, bore: 20.0, key: Key::None, ..Spec::default() },
    );
    case(
        "D flat bore",
        Spec { z: 20, m_n: 1.5, width: 8.0, bore: 8.0, key: Key::DFlat(7.2), ..Spec::default() },
    );
    case(
        "double D bore",
        Spec { z: 20, m_n: 1.5, width: 8.0, bore: 8.0, key: Key::DoubleD(6.4), ..Spec::default() },
    );
    case(
        "helical right hand",
        Spec { z: 31, m_n: 1.5, width: 20.0, beta_deg: 20.0, bore: 12.0, ..Spec::default() },
    );
    case(
        "helical left hand",
        Spec { z: 31, m_n: 1.5, width: 20.0, beta_deg: -20.0, bore: 12.0, ..Spec::default() },
    );
    case(
        "lightening holes",
        Spec {
            z: 60,
            m_n: 25.4 / 12.0,
            width: 6.0,
            bore: 25.0,
            holes: 5,
            hole_dia: 8.0,
            hole_circle: 60.0,
            ..Spec::default()
        },
    );
    case(
        "undercut pinion",
        Spec { z: 10, m_n: 2.0, width: 8.0, bore: 8.0, ..Spec::default() },
    );
    case(
        "shifted pinion",
        Spec { z: 10, m_n: 2.0, width: 8.0, x: 0.45, bore: 8.0, ..Spec::default() },
    );
    case(
        "steep helix, holes",
        Spec {
            z: 40,
            m_n: 2.0,
            width: 30.0,
            beta_deg: 35.0,
            bore: 20.0,
            holes: 6,
            hole_dia: 9.0,
            hole_circle: 55.0,
            ..Spec::default()
        },
    );
    case(
        "fine sampling",
        Spec {
            z: 90,
            m_n: 1.0,
            width: 6.0,
            bore: 20.0,
            flank_seg: 40,
            fillet_seg: 24,
            ..Spec::default()
        },
    );
}
