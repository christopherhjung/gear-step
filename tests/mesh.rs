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
    println!(
        "{:<26} {:>7} tris  volume {:>12.4} vs {:>12.4} mm3  ({:.2e})  {}",
        name,
        mesh.idx.len() / 3,
        vol,
        want,
        err,
        match &res {
            Ok(s) => s.clone(),
            Err(e) => e.clone(),
        }
    );
    res.unwrap_or_else(|e| panic!("{}: {}", name, e));
    assert!(err < 2e-3, "{}: volume off by {:.3}%", name, err * 100.0);
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
