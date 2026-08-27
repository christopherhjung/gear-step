//! Raw WebAssembly interface - no bindgen, no dependencies.
//!
//! The host writes a `Float64Array` of parameters into linear memory, calls
//! `gear_generate`, and then reads the results straight out of memory through
//! the `*_ptr` / `*_len` accessors. Everything stays alive until the next
//! call to `gear_generate`.

use crate::api::{self, Key, Row, Spec};
use crate::brep;

/// Number of f64 slots in the parameter block.
pub const NPARAM: usize = 26;

struct State {
    json: String,
    step: String,
    svg: String,
    pos: Vec<f32>,
    nrm: Vec<f32>,
    idx: Vec<u32>,
    lines: Vec<f32>,
    /// transverse section, xy pairs of every contour back to back
    sec: Vec<f32>,
    /// start index (in points) of each contour, plus the total at the end
    sec_off: Vec<u32>,
}

impl State {
    fn error(msg: &str) -> State {
        State {
            json: format!("{{\"ok\":false,\"error\":{}}}", quote(msg)),
            step: String::new(),
            svg: String::new(),
            pos: Vec::new(),
            nrm: Vec::new(),
            idx: Vec::new(),
            lines: Vec::new(),
            sec: Vec::new(),
            sec_off: Vec::new(),
        }
    }
}

static mut STATE: Option<State> = None;
static mut NAME: String = String::new();

fn state() -> &'static State {
    unsafe {
        match (*core::ptr::addr_of!(STATE)).as_ref() {
            Some(s) => s,
            None => {
                STATE = Some(State::error("nothing generated yet"));
                (*core::ptr::addr_of!(STATE)).as_ref().unwrap()
            }
        }
    }
}

// ---- allocation ------------------------------------------------------------

/// Reserve `len` bytes for the host to write into. Freed with `gear_free`.
#[no_mangle]
pub extern "C" fn gear_alloc(len: usize) -> *mut u8 {
    let mut v: Vec<u8> = Vec::with_capacity(len.max(1));
    let p = v.as_mut_ptr();
    core::mem::forget(v);
    p
}

/// # Safety
/// `ptr` / `len` must come from a previous `gear_alloc`.
#[no_mangle]
pub unsafe extern "C" fn gear_free(ptr: *mut u8, len: usize) {
    if !ptr.is_null() && len > 0 {
        drop(Vec::from_raw_parts(ptr, 0, len.max(1)));
    }
}

/// Set the product name used in the STEP file. Empty means "derive it".
///
/// # Safety
/// `ptr` must point at `len` bytes of valid UTF-8.
#[no_mangle]
pub unsafe extern "C" fn gear_set_name(ptr: *const u8, len: usize) {
    let s = core::slice::from_raw_parts(ptr, len);
    NAME = String::from_utf8_lossy(s).into_owned();
}

// ---- generation ------------------------------------------------------------

fn key_from(mode: f64, b: f64, t2: f64, across: f64) -> Key {
    match mode as i32 {
        0 => Key::None,
        2 => Key::Custom(b, t2),
        3 => Key::DFlat(across),
        4 => Key::DoubleD(across),
        _ => Key::Auto,
    }
}

fn spec_from(p: &[f64]) -> Spec {
    Spec {
        z: p[0].max(0.0) as u32,
        m_n: p[1],
        alpha_n_deg: p[2],
        beta_deg: p[3],
        width: p[4],
        x: p[5],
        ha: p[6],
        hf: p[7],
        rho: p[8],
        backlash: p[9],
        bore: p[10],
        key: key_from(p[11], p[12], p[13], p[14]),
        key_angle_deg: p[15],
        holes: p[16].max(0.0) as u32,
        hole_dia: p[17],
        hole_circle: p[18],
        phase_deg: p[19],
        flank_seg: p[20].max(4.0) as usize,
        fillet_seg: p[21].max(4.0) as usize,
        layers: p[22].max(0.0) as usize,
        pin_dia: p[23],
        name: unsafe { (*core::ptr::addr_of!(NAME)).clone() },
    }
}

/// Build a gear from `NPARAM` f64 values. Returns 1 on success, 0 on a
/// parameter error (the reason is in the JSON block).
///
/// # Safety
/// `ptr` must point at `NPARAM` consecutive `f64` values.
#[no_mangle]
pub unsafe extern "C" fn gear_generate(ptr: *const f64, len: usize) -> u32 {
    if ptr.is_null() || len < NPARAM {
        STATE = Some(State::error("short parameter block"));
        return 0;
    }
    let p = core::slice::from_raw_parts(ptr, len);
    brep::set_epoch(p[24].max(0.0) as i64);
    let want_step = p[25] != 0.0;
    let st = match run(&spec_from(p), want_step) {
        Ok(s) => s,
        Err(e) => {
            STATE = Some(State::error(&e));
            return 0;
        }
    };
    STATE = Some(st);
    1
}

fn run(spec: &Spec, want_step: bool) -> Result<State, String> {
    let built = api::build(spec)?;
    let mesh = built.mesh();

    let mut sec: Vec<f32> = Vec::new();
    let mut sec_off: Vec<u32> = Vec::new();
    for c in built.section() {
        sec_off.push((sec.len() / 2) as u32);
        for q in c {
            sec.push(q[0] as f32);
            sec.push(q[1] as f32);
        }
    }
    sec_off.push((sec.len() / 2) as u32);

    let (step, stats) = if want_step {
        let (t, s) = built.step();
        (t, Some(s))
    } else {
        (String::new(), None)
    };

    let g = &built.g;
    let mut j = String::with_capacity(4096);
    j.push_str("{\"ok\":true,\"name\":");
    j.push_str(&quote(&built.spec.name));
    j.push_str(",\"geom\":{");
    let num = |k: &str, v: f64| format!("\"{}\":{}", k, fmt(v));
    let fields = [
        num("r_pitch", g.r_p),
        num("r_base", g.r_b),
        num("r_tip", g.r_a),
        num("r_root", g.r_f),
        num("r_form", g.r_form),
        num("width", built.p.width),
        num("twist_deg", built.model.twist.to_degrees()),
        num("teeth", built.p.z as f64),
        num("module_t", g.m_t),
        num("bore", built.spec.bore),
        num("alpha_t_deg", g.alpha_t.to_degrees()),
        num("centre_distance", built.g.centre_distance(&built.p).0),
        num("alpha_w_deg", built.g.centre_distance(&built.p).1.to_degrees()),
        num("pair_clearance", built.g.pair_clearance(&built.p)),
        num("backlash_rad", built.g.backlash_angle(&built.p)),
    ];
    j.push_str(&fields.join(","));
    j.push_str("},\"sheet\":[");
    let rows = match &stats {
        Some(s) => built.sheet(s, &s.check(built.genus())),
        None => Vec::new(),
    };
    let mut first = true;
    for r in &rows {
        if !first {
            j.push(',');
        }
        first = false;
        match r {
            Row::Section(s) => {
                j.push_str("{\"section\":");
                j.push_str(&quote(s));
                j.push('}');
            }
            Row::Kv(k, v) => {
                j.push_str("{\"k\":");
                j.push_str(&quote(k));
                j.push_str(",\"v\":");
                j.push_str(&quote(v));
                j.push('}');
            }
        }
    }
    j.push_str("],\"warnings\":[");
    let w = built.warnings();
    for (i, m) in w.iter().enumerate() {
        if i > 0 {
            j.push(',');
        }
        j.push_str(&quote(m));
    }
    j.push_str("],\"counts\":{\"tris\":");
    j.push_str(&(mesh.idx.len() / 3).to_string());
    j.push_str(",\"verts\":");
    j.push_str(&(mesh.pos.len() / 3).to_string());
    j.push_str(",\"step_bytes\":");
    j.push_str(&step.len().to_string());
    j.push_str("}}");

    Ok(State {
        json: j,
        svg: built.svg(),
        step,
        pos: mesh.pos,
        nrm: mesh.nrm,
        idx: mesh.idx,
        lines: mesh.lines,
        sec,
        sec_off,
    })
}

// ---- accessors -------------------------------------------------------------

macro_rules! view {
    ($ptr:ident, $len:ident, $field:ident, $t:ty) => {
        #[no_mangle]
        pub extern "C" fn $ptr() -> *const $t {
            state().$field.as_ptr() as *const $t
        }
        #[no_mangle]
        pub extern "C" fn $len() -> usize {
            state().$field.len()
        }
    };
}

view!(gear_json_ptr, gear_json_len, json, u8);
view!(gear_step_ptr, gear_step_len, step, u8);
view!(gear_svg_ptr, gear_svg_len, svg, u8);
view!(gear_pos_ptr, gear_pos_len, pos, f32);
view!(gear_nrm_ptr, gear_nrm_len, nrm, f32);
view!(gear_idx_ptr, gear_idx_len, idx, u32);
view!(gear_line_ptr, gear_line_len, lines, f32);
view!(gear_sec_ptr, gear_sec_len, sec, f32);
view!(gear_secoff_ptr, gear_secoff_len, sec_off, u32);

#[no_mangle]
pub extern "C" fn gear_nparam() -> usize {
    NPARAM
}

// ---- little JSON helpers ---------------------------------------------------

fn quote(s: &str) -> String {
    let mut o = String::with_capacity(s.len() + 2);
    o.push('"');
    for c in s.chars() {
        match c {
            '"' => o.push_str("\\\""),
            '\\' => o.push_str("\\\\"),
            '\n' => o.push_str("\\n"),
            '\r' => o.push_str("\\r"),
            '\t' => o.push_str("\\t"),
            c if (c as u32) < 0x20 => o.push_str(&format!("\\u{:04x}", c as u32)),
            c => o.push(c),
        }
    }
    o.push('"');
    o
}

fn fmt(v: f64) -> String {
    if v.is_finite() {
        format!("{:.6}", v)
    } else {
        "0".into()
    }
}
