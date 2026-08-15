mod args;
mod brep;
mod gear;
mod keyway;
mod nurbs;
mod profile;
mod svg;

use args::Key;
use gear::GearParams;
use std::f64::consts::PI;

const D2R: f64 = PI / 180.0;

fn main() {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    if argv.is_empty() {
        print!("{}", args::HELP);
        std::process::exit(1);
    }
    match run(&argv) {
        Ok(()) => {}
        Err(e) => {
            eprintln!("error: {}", e);
            std::process::exit(2);
        }
    }
}

fn run(argv: &[String]) -> Result<(), String> {
    let a = match args::parse(argv)? {
        Some(a) => a,
        None => return Ok(()),
    };

    let p = GearParams {
        z: a.z,
        m_n: a.m_n,
        alpha_n: a.alpha_n_deg * D2R,
        beta: a.beta_deg * D2R,
        width: a.width,
        x: a.x,
        ha_c: a.ha,
        hf_c: a.hf,
        rho_c: a.rho,
        backlash: a.backlash,
        flank_seg: a.flank_seg,
        fillet_seg: a.fillet_seg,
    };
    let g = p.build();

    // ---- bore shape ---------------------------------------------------
    let shape = if a.bore > 0.0 {
        match a.key {
            Key::None => keyway::Bore::Round,
            Key::Custom(b, t) => keyway::Bore::Key(b, t),
            Key::DFlat(x) => keyway::Bore::DFlat(x),
            Key::DoubleD(x) => keyway::Bore::DoubleD(x),
            Key::Auto => match keyway::din6885(a.bore) {
                Some((b, t)) => keyway::Bore::Key(b, t),
                None => {
                    eprintln!(
                        "note: no DIN 6885-1 entry for a {:.3} mm bore - plain round bore",
                        a.bore
                    );
                    keyway::Bore::Round
                }
            },
        }
    } else {
        keyway::Bore::Round
    };
    if let Some((h, depth)) = shape.flat(a.bore) {
        if depth <= 0.0 {
            return Err("the flat does not cut into the bore - check --flat".into());
        }
        if h <= -a.bore / 2.0 + 1e-9 {
            return Err("the flat is deeper than the bore".into());
        }
        if h < 0.0 {
            eprintln!("note: the flat cuts past the bore axis");
        }
    }

    // ---- contours -----------------------------------------------------
    let base = g.outline(&p, a.phase_deg * D2R);
    let mut holes: Vec<Vec<profile::Seg>> = Vec::new();
    if a.bore > 0.0 {
        if a.bore / 2.0 >= g.r_f - 0.05 * p.m_n {
            return Err(format!(
                "bore ({:.3} mm) leaves no rim: root diameter is {:.3} mm",
                a.bore,
                2.0 * g.r_f
            ));
        }
        if let keyway::Bore::Key(_, t2) = shape {
            if a.bore / 2.0 + t2 >= g.r_f {
                return Err("keyway breaks through the tooth root".into());
            }
        }
        holes.push(keyway::bore_contour(a.bore, shape, a.key_angle_deg * D2R));
    }
    for i in 0..a.holes {
        let th = 2.0 * PI * i as f64 / a.holes as f64 + a.phase_deg * D2R;
        let rc = a.hole_circle / 2.0;
        holes.push(profile::circle(
            [rc * th.cos(), rc * th.sin()],
            a.hole_dia / 2.0,
        ));
    }

    // ---- sweep ---------------------------------------------------------
    profile::check_closed(&base)?;
    for h in &holes {
        profile::check_closed(h)?;
    }
    let twist = p.width * p.beta.tan() / g.r_p; // total rotation over the face width
    let layers = if a.layers > 0 {
        a.layers.max(4)
    } else {
        (4.0 + (twist.abs() / (30.0 * D2R)).ceil()) as usize
    };
    let model = brep::Model {
        outer: base.clone(),
        holes: holes.clone(),
        z0: 0.0,
        z1: p.width,
        twist,
        layers,
        deg: 3,
    };
    let (text, stats) = brep::write(&model, &a.name);
    let genus = holes.len() as i64;
    let check = stats.check(genus);
    std::fs::write(&a.out, &text).map_err(|e| format!("cannot write {}: {}", a.out, e))?;

    if let Some(path) = &a.svg {
        let poly = |c: &Vec<profile::Seg>| -> Vec<[f64; 2]> {
            let mut v: Vec<[f64; 2]> = Vec::new();
            for s in c {
                for q in s.sample(160) {
                    if v.last().map(|l: &[f64; 2]| (l[0] - q[0]).hypot(l[1] - q[1]) > 1e-9).unwrap_or(true) {
                        v.push(q);
                    }
                }
            }
            v
        };
        let mut cs = vec![poly(&base)];
        cs.extend(holes.iter().map(poly));
        let s = svg::render(
            &cs,
            &[
                (g.r_p, "pitch"),
                (g.r_b, "base"),
                (g.r_f, "root"),
                (g.r_a, "tip"),
            ],
        );
        std::fs::write(path, s).map_err(|e| format!("cannot write {}: {}", path, e))?;
    }

    if !a.quiet {
        report(&a, &p, &g, &stats, &check, shape);
    }
    for w in &g.warnings {
        eprintln!("warning: {}", w);
    }
    if let Err(e) = &check {
        eprintln!("warning: topology check failed: {}", e);
    }
    Ok(())
}

fn report(
    a: &args::Args,
    p: &GearParams,
    g: &gear::Geom,
    stats: &brep::Stats,
    check: &Result<String, String>,
    shape: keyway::Bore,
) {
    let z = p.z as f64;
    let (k, w) = g.span(p);
    let m = g.over_pins(p, a.pin_dia);
    let row = |k: &str, v: String| println!("  {:<34}{}", k, v);
    println!("\n{}", a.name);
    println!("  --- macro geometry ---------------------------------");
    row("teeth z", format!("{}", p.z));
    row("normal module m_n", format!("{:.4} mm", p.m_n));
    row("circular pitch p_t", format!("{:.4} mm", PI * g.m_t));
    row(
        "pressure angle alpha_n / alpha_t",
        format!("{:.4} deg / {:.4} deg", p.alpha_n / D2R, g.alpha_t / D2R),
    );
    row(
        "helix angle beta",
        format!(
            "{:.4} deg {}",
            p.beta.abs() / D2R,
            if p.beta.abs() < 1e-12 {
                "(spur)"
            } else if p.beta > 0.0 {
                "right hand"
            } else {
                "left hand"
            }
        ),
    );
    row("profile shift x", format!("{:.4}", p.x));
    row("face width b", format!("{:.4} mm", p.width));
    println!("  --- diameters --------------------------------------");
    row("pitch d", format!("{:.4} mm", 2.0 * g.r_p));
    row("base d_b", format!("{:.4} mm", 2.0 * g.r_b));
    row("tip d_a", format!("{:.4} mm", 2.0 * g.r_a));
    row("root d_f", format!("{:.4} mm", 2.0 * g.r_f));
    row("form (usable involute) d_Ff", format!("{:.4} mm", 2.0 * g.r_form));
    row("root fillet radius", format!("{:.4} mm", g.rho));
    println!("  --- tooth / inspection -----------------------------");
    row("tooth thickness s_t at d", format!("{:.4} mm", g.s_t));
    row(
        "normal tooth thickness s_n",
        format!("{:.4} mm", g.s_t * p.beta.cos()),
    );
    row(
        "base tangent W over k teeth",
        format!("{:.4} mm (k = {})", w, k),
    );
    row(
        "measurement over pins",
        format!("{:.4} mm (d_M = {:.2} mm)", m, a.pin_dia),
    );
    row("tooth height h", format!("{:.4} mm", g.r_a - g.r_f));
    row(
        "centre distance with equal gear",
        format!("{:.4} mm", 2.0 * g.r_p + 2.0 * p.x * p.m_n),
    );
    if !p.beta.abs().eq(&0.0) {
        row(
            "axial pitch p_x",
            format!("{:.4} mm", PI * p.m_n / p.beta.sin().abs()),
        );
        row(
            "overlap ratio eps_beta",
            format!("{:.3}", p.width * p.beta.sin().abs() / (PI * p.m_n)),
        );
    }
    if a.bore > 0.0 {
        println!("  --- bore -------------------------------------------");
        row("bore d", format!("{:.4} mm (H7 suggested)", a.bore));
        match shape {
            keyway::Bore::Key(b, t2) => {
                row("keyway b x t2", format!("{:.2} x {:.2} mm", b, t2));
                row("dimension d + t2", format!("{:.3} mm", a.bore + t2));
                row("rim under the groove", format!("{:.3} mm", g.r_f - a.bore / 2.0 - t2));
            }
            keyway::Bore::DFlat(across) => {
                let (h, depth) = shape.flat(a.bore).unwrap();
                row("D profile across flat", format!("{:.3} mm", across));
                row("flat depth / chord", format!(
                    "{:.3} mm / {:.3} mm",
                    depth,
                    2.0 * ((a.bore / 2.0).powi(2) - h * h).max(0.0).sqrt()
                ));
            }
            keyway::Bore::DoubleD(across) => {
                let (h, depth) = shape.flat(a.bore).unwrap();
                row("double D across flats", format!("{:.3} mm", across));
                row("flat depth each / chord", format!(
                    "{:.3} mm / {:.3} mm",
                    depth,
                    2.0 * ((a.bore / 2.0).powi(2) - h * h).max(0.0).sqrt()
                ));
            }
            keyway::Bore::Round => row("bore profile", "plain round".into()),
        }
        row("rim width (root to bore)", format!("{:.3} mm", g.r_f - a.bore / 2.0));
    }
    println!("  --- output -----------------------------------------");
    row(
        "faces / edges / vertices",
        format!("{} / {} / {}", stats.faces, stats.edges, stats.verts),
    );
    row(
        "surfaces",
        format!(
            "{} planar, {} cylindrical, {} B-spline",
            stats.planar, stats.cylindrical, stats.spline
        ),
    );
    row("STEP entities", format!("{}", stats.entities));
    row(
        "flank spline deviation",
        format!("{:.4} um max vs exact involute", g.flank_deviation(3) * 1000.0),
    );
    row(
        "topology",
        match check {
            Ok(s) => s.clone(),
            Err(e) => format!("FAILED: {}", e),
        },
    );
    row("file", a.out.clone());
    let _ = z;
    println!();
}
