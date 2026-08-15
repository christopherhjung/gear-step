//! Quick 2D dump of the transverse section (outer contour + bore + holes).

pub fn render(contours: &[Vec<[f64; 2]>], r_ref: &[(f64, &str)]) -> String {
    let mut max = 1.0f64;
    for c in contours {
        for p in c {
            max = max.max(p[0].hypot(p[1]));
        }
    }
    for (r, _) in r_ref {
        max = max.max(*r);
    }
    let s = max * 1.06;
    let mut out = format!(
        "<svg xmlns='http://www.w3.org/2000/svg' viewBox='{:.3} {:.3} {:.3} {:.3}' width='900' height='900'>\n<g transform='scale(1,-1)'>\n",
        -s,
        -s,
        2.0 * s,
        2.0 * s
    );
    let lw = s / 400.0;
    for (r, class) in r_ref {
        let dash = match *class {
            "pitch" => format!("{} {}", 6.0 * lw, 3.0 * lw),
            _ => format!("{} {}", 2.0 * lw, 2.0 * lw),
        };
        out += &format!(
            "<circle cx='0' cy='0' r='{:.5}' fill='none' stroke='#c00' stroke-width='{:.5}' stroke-dasharray='{}'/>\n",
            r, lw, dash
        );
    }
    for c in contours {
        let pts: Vec<String> = c.iter().map(|p| format!("{:.5},{:.5}", p[0], p[1])).collect();
        out += &format!(
            "<polygon points='{}' fill='#0a58ca22' stroke='#123' stroke-width='{:.5}'/>\n",
            pts.join(" "),
            lw * 1.5
        );
    }
    out += "</g>\n</svg>\n";
    out
}
