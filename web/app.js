// gear-step in the browser: the Rust generator compiled to WebAssembly, a
// WebGL viewer for the display mesh, and the same data sheet the CLI prints.

// ---------------------------------------------------------------- wasm glue

const PARAM_KEYS = [
  'z', 'm_n', 'alpha', 'beta', 'width', 'shift', 'ha', 'hf', 'rho', 'backlash',
  'bore', 'keymode', 'key_b', 'key_t2', 'across', 'boreang', 'holes', 'holedia',
  'holecircle', 'phase', 'flankseg', 'filletseg', 'layers', 'pin', 'epoch', 'wantstep',
];

class Gear {
  async load() {
    let bytes;
    if (typeof GEAR_WASM_B64 === 'string') {           // single file build
      const bin = atob(GEAR_WASM_B64);
      bytes = new Uint8Array(bin.length);
      for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
    } else {
      bytes = await (await fetch('gear.wasm')).arrayBuffer();
    }
    const { instance } = await WebAssembly.instantiate(bytes, {});
    this.e = instance.exports;
    this.n = this.e.gear_nparam();
    this.pptr = this.e.gear_alloc(this.n * 8);
    this.enc = new TextEncoder();
    this.dec = new TextDecoder();
    return this;
  }
  get buf() { return this.e.memory.buffer; }
  f32(ptr, len) { return new Float32Array(this.buf, ptr, len); }
  u32(ptr, len) { return new Uint32Array(this.buf, ptr, len); }
  text(ptr, len) { return this.dec.decode(new Uint8Array(this.buf, ptr, len)); }

  setName(s) {
    const b = this.enc.encode(s);
    const p = this.e.gear_alloc(b.length + 1);
    new Uint8Array(this.buf, p, b.length).set(b);
    this.e.gear_set_name(p, b.length);
    this.e.gear_free(p, b.length + 1);
  }

  // params: object keyed like PARAM_KEYS
  generate(params) {
    this.setName(params.name || '');
    const a = new Float64Array(this.buf, this.pptr, this.n);
    PARAM_KEYS.forEach((k, i) => { a[i] = Number(params[k]) || 0; });
    const ok = this.e.gear_generate(this.pptr, this.n);
    const e = this.e;
    const json = JSON.parse(this.text(e.gear_json_ptr(), e.gear_json_len()));
    if (!ok) return json;
    json.mesh = {
      pos: this.f32(e.gear_pos_ptr(), e.gear_pos_len()).slice(),
      nrm: this.f32(e.gear_nrm_ptr(), e.gear_nrm_len()).slice(),
      idx: this.u32(e.gear_idx_ptr(), e.gear_idx_len()).slice(),
      lines: this.f32(e.gear_line_ptr(), e.gear_line_len()).slice(),
    };
    const sec = this.f32(e.gear_sec_ptr(), e.gear_sec_len());
    const off = this.u32(e.gear_secoff_ptr(), e.gear_secoff_len());
    json.section = [];
    for (let i = 0; i + 1 < off.length; i++) {
      json.section.push(sec.slice(off[i] * 2, off[i + 1] * 2));
    }
    json.step = () => this.text(e.gear_step_ptr(), e.gear_step_len());
    json.svg = () => this.text(e.gear_svg_ptr(), e.gear_svg_len());
    return json;
  }
}

// ------------------------------------------------------------ small matrix

const M4 = {
  ident: () => new Float32Array([1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]),
  mul(a, b) {
    const o = new Float32Array(16);
    for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) {
      let s = 0;
      for (let k = 0; k < 4; k++) s += a[k * 4 + r] * b[c * 4 + k];
      o[c * 4 + r] = s;
    }
    return o;
  },
  persp(fovy, asp, n, f) {
    const t = 1 / Math.tan(fovy / 2);
    return new Float32Array([
      t / asp, 0, 0, 0,
      0, t, 0, 0,
      0, 0, (f + n) / (n - f), -1,
      0, 0, 2 * f * n / (n - f), 0]);
  },
  lookAt(eye, at, up) {
    const z = norm(sub(eye, at)), x = norm(cross(up, z)), y = cross(z, x);
    return new Float32Array([
      x[0], y[0], z[0], 0,
      x[1], y[1], z[1], 0,
      x[2], y[2], z[2], 0,
      -dot(x, eye), -dot(y, eye), -dot(z, eye), 1]);
  },
  trans(x, y, z) { const m = M4.ident(); m[12] = x; m[13] = y; m[14] = z; return m; },
  scale(x, y, z) { const m = M4.ident(); m[0] = x; m[5] = y; m[10] = z; return m; },
  rotZ(a) {
    const m = M4.ident(), c = Math.cos(a), s = Math.sin(a);
    m[0] = c; m[1] = s; m[4] = -s; m[5] = c; return m;
  },
};
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = (a) => { const l = Math.hypot(...a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };

// ------------------------------------------------------------- gl renderer

const FOV = 0.72;   // vertical field of view, radians

const VS = `#version 300 es
precision highp float;
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNrm;
uniform mat4 uMVP, uModel;
uniform mat3 uNrmMat;
out vec3 vN; out vec3 vP;
void main(){
  vN = normalize(uNrmMat * aNrm);
  vP = (uModel * vec4(aPos,1.0)).xyz;
  gl_Position = uMVP * vec4(aPos,1.0);
}`;

const FS = `#version 300 es
precision highp float;
in vec3 vN; in vec3 vP;
uniform vec3 uEye;
uniform vec3 uBase;
out vec4 frag;

vec3 aces(vec3 x){
  return clamp((x*(2.51*x+0.03))/(x*(2.43*x+0.59)+0.14), 0.0, 1.0);
}

void main(){
  vec3 N = normalize(vN);
  if (!gl_FrontFacing) N = -N;
  vec3 V = normalize(uEye - vP);

  vec3 key  = normalize(vec3( 0.78, 0.50, 0.38));
  vec3 fill = normalize(vec3(-0.85, 0.05, 0.15));
  vec3 rim  = normalize(vec3( 0.10,-0.90,-0.10));

  // hemisphere ambient: cool sky above, near black below
  vec3 amb = mix(vec3(0.010,0.014,0.022), vec3(0.16,0.21,0.29), N.z*0.5+0.5);

  vec3 col = uBase * (amb + 1.30*max(dot(N,key),0.0) + 0.30*max(dot(N,fill),0.0));
  col += uBase * vec3(0.45,0.55,0.75) * 0.34 * max(dot(N,rim),0.0);

  // two tight highlights, for the machined look
  col += vec3(1.00,0.96,0.88) * 0.75 * pow(max(dot(N,normalize(key+V)),0.0), 90.0);
  col += vec3(0.65,0.78,1.00) * 0.22 * pow(max(dot(N,normalize(fill+V)),0.0), 20.0);

  // grazing sheen picks out the tooth flanks
  col += vec3(0.30,0.40,0.58) * 0.35 * pow(1.0 - max(dot(N,V),0.0), 4.0);

  frag = vec4(pow(aces(col), vec3(1.0/2.2)), 1.0);
}`;

const LVS = `#version 300 es
layout(location=0) in vec3 aPos;
uniform mat4 uMVP;
void main(){
  gl_Position = uMVP * vec4(aPos,1.0);
}`;
const LFS = `#version 300 es
precision highp float;
uniform vec4 uCol;
out vec4 frag;
void main(){ frag = uCol; }`;

class View {
  constructor(canvas) {
    this.cv = canvas;
    const gl = canvas.getContext('webgl2', { antialias: true, alpha: false });
    if (!gl) throw new Error('WebGL2 is not available in this browser');
    this.gl = gl;
    gl.enable(gl.DEPTH_TEST);
    gl.enable(gl.CULL_FACE);
    this.prog = this.link(VS, FS);
    this.lprog = this.link(LVS, LFS);
    this.u = {
      mvp: gl.getUniformLocation(this.prog, 'uMVP'),
      model: gl.getUniformLocation(this.prog, 'uModel'),
      nrm: gl.getUniformLocation(this.prog, 'uNrmMat'),
      eye: gl.getUniformLocation(this.prog, 'uEye'),
      base: gl.getUniformLocation(this.prog, 'uBase'),
    };
    this.lu = {
      mvp: gl.getUniformLocation(this.lprog, 'uMVP'),
      col: gl.getUniformLocation(this.lprog, 'uCol'),
    };
    this.vao = gl.createVertexArray();
    this.vbo = gl.createBuffer();
    this.nbo = gl.createBuffer();
    this.ibo = gl.createBuffer();
    this.lvao = gl.createVertexArray();
    this.lbo = gl.createBuffer();
    this.count = 0;
    this.lineCount = 0;

    // camera
    this.az = -1.02; this.el = 0.52; this.dist = 100;
    this.centre = [0, 0, 0];
    this.teeth = 0;
    this.target = [0, 0, 0];
    this.framed = false;
    this.radius = 30;
    this.spin = 0;
    this.opts = { edges: true, spin: false, pair: false };
    this.pairDist = 0;
    this.dirty = true;
    this.hookInput();
  }
  link(vs, fs) {
    const gl = this.gl;
    const mk = (t, src) => {
      const s = gl.createShader(t);
      gl.shaderSource(s, src); gl.compileShader(s);
      if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s));
      return s;
    };
    const p = gl.createProgram();
    gl.attachShader(p, mk(gl.VERTEX_SHADER, vs));
    gl.attachShader(p, mk(gl.FRAGMENT_SHADER, fs));
    gl.linkProgram(p);
    if (!gl.getProgramParameter(p, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(p));
    return p;
  }
  upload(mesh, geom) {
    const gl = this.gl;
    gl.bindVertexArray(this.vao);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.vbo);
    gl.bufferData(gl.ARRAY_BUFFER, mesh.pos, gl.STATIC_DRAW);
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.nbo);
    gl.bufferData(gl.ARRAY_BUFFER, mesh.nrm, gl.STATIC_DRAW);
    gl.enableVertexAttribArray(1);
    gl.vertexAttribPointer(1, 3, gl.FLOAT, false, 0, 0);
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, this.ibo);
    gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, mesh.idx, gl.STATIC_DRAW);
    this.count = mesh.idx.length;

    gl.bindVertexArray(this.lvao);
    gl.bindBuffer(gl.ARRAY_BUFFER, this.lbo);
    gl.bufferData(gl.ARRAY_BUFFER, mesh.lines, gl.STATIC_DRAW);
    gl.enableVertexAttribArray(0);
    gl.vertexAttribPointer(0, 3, gl.FLOAT, false, 0, 0);
    this.lineCount = mesh.lines.length / 3;
    gl.bindVertexArray(null);

    this.centre = [0, 0, geom.width / 2];
    this.teeth = geom.teeth;
    this.radius = Math.hypot(geom.r_tip, geom.width / 2);
    if (!this.framed) { this.target = this.centre.slice(); this.framed = true; }
    this.dirty = true;
  }
  fit() {
    const c = this.centre || [0, 0, 0];
    const pair = this.opts.pair && this.pairDist > 0;
    // a meshing pair sits between the two axes
    this.target = [c[0] + (pair ? this.pairDist / 2 : 0), c[1], c[2]];
    // bounding sphere of what is on screen, fitted to the narrower field
    const span = this.span();
    const asp = Math.max(this.cv.clientWidth, 1) / Math.max(this.cv.clientHeight, 1);
    const half = Math.min(FOV / 2, Math.atan(Math.tan(FOV / 2) * asp));
    this.dist = span / Math.sin(half) * 1.05;
    this.dirty = true;
  }
  /// Radius of the bounding sphere around the camera target.
  span() {
    const pair = this.opts.pair && this.pairDist > 0;
    return pair ? this.pairDist / 2 + this.radius : this.radius;
  }
  hookInput() {
    const cv = this.cv;
    let drag = null;
    cv.addEventListener('pointerdown', (e) => {
      drag = { x: e.clientX, y: e.clientY, pan: e.shiftKey || e.button === 1 || e.button === 2 };
      cv.setPointerCapture(e.pointerId);
    });
    cv.addEventListener('contextmenu', (e) => e.preventDefault());
    cv.addEventListener('pointermove', (e) => {
      if (!drag) return;
      const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
      drag.x = e.clientX; drag.y = e.clientY;
      if (drag.pan) {
        const k = this.dist * 0.0016;
        const right = [Math.cos(this.az + Math.PI / 2), Math.sin(this.az + Math.PI / 2), 0];
        const up = this.upVec();
        for (let i = 0; i < 3; i++) this.target[i] += -right[i] * dx * k + up[i] * dy * k;
      } else {
        this.az -= dx * 0.008;
        this.el = Math.max(-1.5, Math.min(1.5, this.el + dy * 0.008));
      }
      this.dirty = true;
    });
    const stop = (e) => { if (drag) { drag = null; cv.releasePointerCapture?.(e.pointerId); } };
    cv.addEventListener('pointerup', stop);
    cv.addEventListener('pointercancel', stop);
    cv.addEventListener('wheel', (e) => {
      e.preventDefault();
      this.dist *= Math.exp(e.deltaY * 0.0012);
      this.dist = Math.max(this.radius * 0.35, Math.min(this.radius * 60, this.dist));
      this.dirty = true;
    }, { passive: false });
  }
  upVec() {
    const f = norm(sub(this.eye(), this.target));
    const right = [Math.cos(this.az + Math.PI / 2), Math.sin(this.az + Math.PI / 2), 0];
    return cross(f, right);
  }
  eye() {
    const ce = Math.cos(this.el);
    return [
      this.target[0] + this.dist * ce * Math.cos(this.az),
      this.target[1] + this.dist * ce * Math.sin(this.az),
      this.target[2] + this.dist * Math.sin(this.el),
    ];
  }
  resize() {
    const dpr = Math.min(devicePixelRatio || 1, 2);
    const w = Math.round(this.cv.clientWidth * dpr), h = Math.round(this.cv.clientHeight * dpr);
    if (this.cv.width !== w || this.cv.height !== h) {
      this.cv.width = w; this.cv.height = h;
      this.dirty = true;
    }
  }
  draw(dt) {
    this.resize();
    if (this.opts.spin) { this.spin += dt * 0.55; this.dirty = true; }
    if (!this.dirty) return;
    this.dirty = false;
    const gl = this.gl;
    const w = this.cv.width, h = this.cv.height;
    gl.viewport(0, 0, w, h);
    gl.clearColor(0.043, 0.055, 0.075, 1);
    gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
    if (!this.count) return;

    const eye = this.eye();
    // Keep the depth range as tight as the scene allows: a slack far plane is
    // what makes edges poke through the faces in front of them.
    const span = this.span() * 1.25;
    const near = Math.max(this.dist - span, span * 0.01);
    const proj = M4.persp(FOV, w / h, near, this.dist + span);
    const view = M4.lookAt(eye, this.target, [0, 0, 1]);
    const vp = M4.mul(proj, view);

    const drawOne = (model, base, mirrored) => {
      gl.frontFace(mirrored ? gl.CW : gl.CCW);
      const mvp = M4.mul(vp, model);
      gl.useProgram(this.prog);
      gl.uniformMatrix4fv(this.u.mvp, false, mvp);
      gl.uniformMatrix4fv(this.u.model, false, model);
      gl.uniformMatrix3fv(this.u.nrm, false, new Float32Array([
        model[0], model[1], model[2],
        model[4], model[5], model[6],
        model[8], model[9], model[10]]));
      gl.uniform3fv(this.u.eye, new Float32Array(eye));
      gl.uniform3fv(this.u.base, new Float32Array(base));
      gl.bindVertexArray(this.vao);
      // push the filled faces a hair away from the eye so the edges that lie
      // exactly on them win, without lifting the ones behind them
      gl.enable(gl.POLYGON_OFFSET_FILL);
      gl.polygonOffset(1.0, 1.0);
      gl.drawElements(gl.TRIANGLES, this.count, gl.UNSIGNED_INT, 0);
      gl.disable(gl.POLYGON_OFFSET_FILL);
      if (this.opts.edges && this.lineCount) {
        gl.useProgram(this.lprog);
        gl.uniformMatrix4fv(this.lu.mvp, false, mvp);
        gl.uniform4f(this.lu.col, 0.02, 0.03, 0.05, 0.5);
        gl.enable(gl.BLEND);
        gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
        gl.bindVertexArray(this.lvao);
        gl.drawArrays(gl.LINES, 0, this.lineCount);
        gl.disable(gl.BLEND);
      }
      gl.bindVertexArray(null);
    };

    const steel = [0.30, 0.335, 0.395];
    const brass = [0.50, 0.345, 0.13];
    drawOne(M4.rotZ(this.spin), steel, false);
    if (this.opts.pair && this.pairDist > 0 && this.teeth > 0) {
      // The mate is this gear mirrored about the plane x = a/2, turned by half
      // a pitch so its teeth land in these gaps instead of on them. The mirror
      // makes it counter-rotate on its own and, for a helical gear, gives it
      // the opposite hand - which is what a parallel axis pair needs.
      const m = M4.mul(M4.trans(this.pairDist, 0, 0),
        M4.mul(M4.scale(-1, 1, 1), M4.rotZ(this.spin + Math.PI / this.teeth)));
      drawOne(m, brass, true);
    }
    gl.frontFace(gl.CCW);
  }
}

// -------------------------------------------------------- 2d profile view

class Profile {
  constructor(cv) {
    this.cv = cv; this.ctx = cv.getContext('2d');
    this.zoom = 1; this.ox = 0; this.oy = 0;
    this.data = null; this.dirty = true;
    let drag = null;
    cv.addEventListener('pointerdown', (e) => { drag = [e.clientX, e.clientY]; cv.setPointerCapture(e.pointerId); });
    cv.addEventListener('pointermove', (e) => {
      if (!drag) return;
      this.ox += e.clientX - drag[0]; this.oy += e.clientY - drag[1];
      drag = [e.clientX, e.clientY]; this.dirty = true;
    });
    cv.addEventListener('pointerup', () => { drag = null; });
    cv.addEventListener('wheel', (e) => {
      e.preventDefault();
      this.zoom *= Math.exp(-e.deltaY * 0.0012);
      this.zoom = Math.max(0.15, Math.min(60, this.zoom));
      this.dirty = true;
    }, { passive: false });
  }
  set(res) { this.data = res; this.dirty = true; }
  reset() { this.zoom = 1; this.ox = 0; this.oy = 0; this.dirty = true; }
  draw() {
    const dpr = Math.min(devicePixelRatio || 1, 2);
    const w = Math.round(this.cv.clientWidth * dpr), h = Math.round(this.cv.clientHeight * dpr);
    if (this.cv.width !== w || this.cv.height !== h) { this.cv.width = w; this.cv.height = h; this.dirty = true; }
    if (!this.dirty) return;
    this.dirty = false;
    const g = this.ctx;
    g.setTransform(1, 0, 0, 1, 0, 0);
    g.fillStyle = '#0b0e13';
    g.fillRect(0, 0, w, h);
    if (!this.data) return;
    const { section, geom } = this.data;
    const s = Math.min(w, h) / (2.25 * geom.r_tip) * this.zoom;
    g.translate(w / 2 + this.ox * dpr, h / 2 + this.oy * dpr);
    g.scale(s, -s);
    const lw = 1.3 / s;

    const circle = (r, col, dash) => {
      g.beginPath(); g.arc(0, 0, r, 0, Math.PI * 2);
      g.strokeStyle = col; g.lineWidth = lw; g.setLineDash(dash.map((d) => d / s)); g.stroke();
      g.setLineDash([]);
    };
    circle(geom.r_tip, '#3a4756', [4, 4]);
    circle(geom.r_root, '#3a4756', [4, 4]);
    circle(geom.r_base, '#2f6f8f', [2, 3]);
    circle(geom.r_form, '#2f6f4f', [1, 3]);
    circle(geom.r_pitch, '#c0392b', [9, 4, 2, 4]);

    g.beginPath();
    section.forEach((c) => {
      for (let i = 0; i < c.length; i += 2) {
        if (i === 0) g.moveTo(c[0], c[1]); else g.lineTo(c[i], c[i + 1]);
      }
      g.closePath();
    });
    g.fillStyle = '#4ea8de26';
    g.fill('evenodd');
    g.strokeStyle = '#d8e2ef';
    g.lineWidth = lw;
    g.lineJoin = 'round';
    g.stroke();

    // centre mark
    g.beginPath();
    g.moveTo(-geom.r_tip * 0.12, 0); g.lineTo(geom.r_tip * 0.12, 0);
    g.moveTo(0, -geom.r_tip * 0.12); g.lineTo(0, geom.r_tip * 0.12);
    g.strokeStyle = '#c0392b'; g.lineWidth = lw; g.stroke();
  }
}

// ------------------------------------------------------------------- state

const $ = (id) => document.getElementById(id);

const PRESETS = {
  'spur pinion, keyed 20 mm bore':
    { z: 24, size: 2, sizeMode: 'module', width: 12, bore: 20, keymode: 1 },
  'undercut pinion, z = 10':
    { z: 10, size: 2, sizeMode: 'module', width: 8, bore: 8, keymode: 1, shift: 0 },
  'the same, corrected by x = 0.45':
    { z: 10, size: 2, sizeMode: 'module', width: 8, bore: 8, keymode: 1, shift: 0.45 },
  'helical wheel, 20 deg RH':
    { z: 31, size: 1.5, sizeMode: 'module', width: 20, beta: 20, hand: 'right', bore: 12, backlash: 0.06, keymode: 1 },
  'D bore for an 8 mm motor shaft':
    { z: 20, size: 1.5, sizeMode: 'module', width: 8, bore: 8, keymode: 3, across: 7.2 },
  '12 DP, 60 teeth, lightening holes':
    { z: 60, size: 12, sizeMode: 'dp', width: 6, bore: 25, keymode: 1, holes: 5, holedia: 8, holecircle: 60 },
  'high helix, 35 deg, 6 holes':
    { z: 40, size: 2, sizeMode: 'module', width: 30, beta: 35, bore: 20, keymode: 1, holes: 6, holedia: 9, holecircle: 55 },
};

const state = {
  sizeMode: 'module', hand: 'right', name: '',
  z: 24, size: 2, width: 12, alpha: 20, beta: 0, shift: 0, backlash: 0,
  ha: 1, hf: 1.25, rho: 0.38,
  bore: 20, keymode: 1, key_b: 6, key_t2: 2.8, across: 18, boreang: 0,
  holes: 0, holedia: 8, holecircle: 34,
  phase: 0, flankseg: 16, filletseg: 10, layers: 0, pin: 0,
};

const NUMS = ['z', 'size', 'width', 'alpha', 'beta', 'shift', 'backlash', 'ha', 'hf', 'rho',
  'bore', 'key_b', 'key_t2', 'across', 'boreang', 'holes', 'holedia', 'holecircle',
  'phase', 'flankseg', 'filletseg', 'layers', 'pin'];

function moduleFromSize() {
  const v = state.size;
  if (state.sizeMode === 'pitch') return v / Math.PI;
  if (state.sizeMode === 'dp') return 25.4 / v;
  return v;
}

function specOf() {
  return {
    z: state.z,
    m_n: moduleFromSize(),
    alpha: state.alpha,
    beta: state.hand === 'left' ? -Math.abs(state.beta) : Math.abs(state.beta),
    width: state.width,
    shift: state.shift,
    ha: state.ha, hf: state.hf, rho: state.rho,
    backlash: state.backlash,
    bore: state.bore,
    keymode: state.bore > 0 ? state.keymode : 0,
    key_b: state.key_b, key_t2: state.key_t2, across: state.across,
    boreang: state.boreang,
    holes: state.holes, holedia: state.holedia, holecircle: state.holecircle,
    phase: state.phase,
    flankseg: state.flankseg, filletseg: state.filletseg,
    layers: state.layers, pin: state.pin,
    epoch: Math.floor(Date.now() / 1000),
    wantstep: 1,
    name: state.name,
  };
}

// ---------------------------------------------------------------- ui wiring

function linkField(key) {
  const num = $(key), rng = $(key + '_r');
  if (!num) return;
  const push = (v, from) => {
    const x = Number(v);
    if (!Number.isFinite(x)) return;
    state[key] = x;
    if (rng && from !== 'r') rng.value = x;
    if (from !== 'n') num.value = x;
    schedule();
  };
  num.addEventListener('input', () => push(num.value, 'n'));
  if (rng) rng.addEventListener('input', () => push(rng.value, 'r'));
}

function setField(key, v) {
  state[key] = v;
  const num = $(key), rng = $(key + '_r');
  if (num) num.value = v;
  if (rng) rng.value = v;
}

function segGroup(id, onPick) {
  const el = $(id);
  el.addEventListener('click', (e) => {
    const b = e.target.closest('button');
    if (!b) return;
    [...el.querySelectorAll('button')].forEach((x) => x.setAttribute('aria-pressed', String(x === b)));
    onPick(b.dataset.v);
  });
  return (v) => {
    [...el.querySelectorAll('button')].forEach((x) => x.setAttribute('aria-pressed', String(x.dataset.v === v)));
  };
}

function syncSizeLabels() {
  const l = { module: ['normal module m_n', 'mm'], pitch: ['circular pitch p', 'mm'], dp: ['diametral pitch', '1/in'] };
  const [a, b] = l[state.sizeMode];
  $('sizeLabel').textContent = a;
  $('sizeUnit').textContent = b;
  const r = $('size_r');
  if (state.sizeMode === 'dp') { r.min = 1; r.max = 64; r.step = 0.5; }
  else { r.min = 0.2; r.max = 10; r.step = 0.05; }
  r.value = state.size;
}

function syncBoreUi() {
  const m = Number(state.keymode);
  $('keyCustom').hidden = m !== 2;
  $('keyFlat').hidden = !(m === 3 || m === 4);
  $('flatLabel').textContent = m === 4 ? 'between the two flats' : 'flat to opposite wall';
}

// ------------------------------------------------------------------- output

function renderSheet(res) {
  $('sheetName').textContent = res.name || '-';
  const rows = $('rows');
  rows.innerHTML = '';
  for (const r of res.sheet || []) {
    if (r.section) {
      const d = document.createElement('div');
      d.className = 'sec-title'; d.textContent = r.section;
      rows.appendChild(d);
    } else {
      const d = document.createElement('div');
      d.className = 'kv';
      const k = document.createElement('span'); k.className = 'k'; k.textContent = r.k;
      const v = document.createElement('span'); v.className = 'v'; v.textContent = r.v;
      d.append(k, v); rows.appendChild(d);
    }
  }
}

function renderWarnings(list, err) {
  const box = $('warn');
  box.innerHTML = '';
  if (err) {
    const d = document.createElement('div');
    d.className = 'warning err'; d.textContent = err;
    box.appendChild(d);
  }
  for (const w of list || []) {
    const d = document.createElement('div');
    d.className = 'warning'; d.textContent = w;
    box.appendChild(d);
  }
}

function download(name, data, mime) {
  const blob = new Blob([data], { type: mime });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url; a.download = name;
  document.body.appendChild(a); a.click(); a.remove();
  setTimeout(() => URL.revokeObjectURL(url), 4000);
}

/// Binary STL straight from the display mesh.
function stlOf(mesh, name) {
  const n = mesh.idx.length / 3;
  const buf = new ArrayBuffer(84 + n * 50);
  const dv = new DataView(buf);
  const hdr = new TextEncoder().encode(('binary STL from gear-step: ' + name).slice(0, 79));
  new Uint8Array(buf, 0, 80).set(hdr);
  dv.setUint32(80, n, true);
  let o = 84;
  const p = mesh.pos;
  for (let t = 0; t < n; t++) {
    const a = mesh.idx[t * 3] * 3, b = mesh.idx[t * 3 + 1] * 3, c = mesh.idx[t * 3 + 2] * 3;
    const ux = p[b] - p[a], uy = p[b + 1] - p[a + 1], uz = p[b + 2] - p[a + 2];
    const vx = p[c] - p[a], vy = p[c + 1] - p[a + 1], vz = p[c + 2] - p[a + 2];
    let nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    const l = Math.hypot(nx, ny, nz) || 1;
    dv.setFloat32(o, nx / l, true); dv.setFloat32(o + 4, ny / l, true); dv.setFloat32(o + 8, nz / l, true);
    o += 12;
    for (const i of [a, b, c]) {
      dv.setFloat32(o, p[i], true); dv.setFloat32(o + 4, p[i + 1], true); dv.setFloat32(o + 8, p[i + 2], true);
      o += 12;
    }
    dv.setUint16(o, 0, true); o += 2;
  }
  return buf;
}

// ------------------------------------------------------------------- driver

let gear, view, profile, last = null, pending = false;

function schedule() {
  if (pending) return;
  pending = true;
  requestAnimationFrame(() => { pending = false; regenerate(); });
}

function regenerate() {
  const t0 = performance.now();
  let res;
  try {
    res = gear.generate(specOf());
  } catch (e) {
    renderWarnings([], String(e && e.message || e));
    return;
  }
  if (!res.ok) {
    renderWarnings([], res.error);
    $('stats').textContent = 'no solid: ' + res.error;
    return;
  }
  last = res;
  view.upload(res.mesh, res.geom);
  view.pairDist = 2 * res.geom.r_pitch + 2 * state.shift * moduleFromSize();
  profile.set(res);
  renderSheet(res);
  renderWarnings(res.warnings, null);
  const ms = performance.now() - t0;
  $('stats').textContent =
    `${res.counts.tris.toLocaleString()} triangles · ${(res.counts.step_bytes / 1024).toFixed(0)} kB STEP · ${ms.toFixed(0)} ms`;
  writeHash();
}

// ---- url state -------------------------------------------------------------

const HASH_KEYS = [...NUMS, 'sizeMode', 'hand', 'name'];

function writeHash() {
  const d = { ...state };
  const parts = [];
  for (const k of HASH_KEYS) {
    if (d[k] === '' || d[k] === undefined) continue;
    parts.push(`${k}=${encodeURIComponent(d[k])}`);
  }
  history.replaceState(null, '', '#' + parts.join('&'));
}

function readHash() {
  const h = location.hash.replace(/^#/, '');
  if (!h) return false;
  let any = false;
  for (const kv of h.split('&')) {
    const [k, v] = kv.split('=');
    if (!(k in state)) continue;
    state[k] = (k === 'sizeMode' || k === 'hand' || k === 'name') ? decodeURIComponent(v) : Number(v);
    any = true;
  }
  return any;
}

function pushStateToUi() {
  for (const k of NUMS) setField(k, state[k]);
  $('keymode').value = String(state.keymode);
  $('name').value = state.name || '';
  setSize(state.sizeMode); setHand(state.hand);
  syncSizeLabels(); syncBoreUi();
}

let setSize, setHand;

function applyPreset(p) {
  Object.assign(state, {
    // reset the fields a preset does not mention
    alpha: 20, beta: 0, shift: 0, backlash: 0, ha: 1, hf: 1.25, rho: 0.38,
    holes: 0, boreang: 0, phase: 0, keymode: 1, hand: 'right', sizeMode: 'module',
  }, p);
  pushStateToUi();
  schedule();
}

function buildPresets() {
  const box = $('presets');
  for (const [name, p] of Object.entries(PRESETS)) {
    const b = document.createElement('button');
    b.className = 'chip';
    b.style.cssText = 'display:block;width:100%;text-align:left;margin:5px 0;border-radius:5px';
    b.textContent = name;
    b.onclick = () => applyPreset(p);
    box.appendChild(b);
  }
}

function toggleChip(id, key) {
  const el = $(id);
  el.addEventListener('click', () => {
    view.opts[key] = !view.opts[key];
    el.setAttribute('aria-pressed', String(view.opts[key]));
    if (key === 'pair') view.fit();
    view.dirty = true;
  });
}

async function main() {
  gear = await new Gear().load();
  try {
    view = new View($('gl'));
  } catch (e) {
    $('stats').textContent = e.message;
    document.body.classList.add('mode-2d');
  }
  profile = new Profile($('profile'));

  NUMS.forEach(linkField);
  setSize = segGroup('sizeMode', (v) => {
    // keep the physical tooth size when switching how it is written down
    const m = moduleFromSize();
    state.sizeMode = v;
    const nv = v === 'pitch' ? m * Math.PI : v === 'dp' ? 25.4 / m : m;
    setField('size', Math.round(nv * 1000) / 1000);
    syncSizeLabels(); schedule();
  });
  setHand = segGroup('hand', (v) => { state.hand = v; schedule(); });
  $('keymode').addEventListener('change', (e) => {
    state.keymode = Number(e.target.value); syncBoreUi(); schedule();
  });
  $('name').addEventListener('input', (e) => { state.name = e.target.value; schedule(); });

  buildPresets();
  toggleChip('tEdges', 'edges');
  toggleChip('tSpin', 'spin');
  toggleChip('tPair', 'pair');
  $('tFit').addEventListener('click', () => {
    if (document.body.classList.contains('mode-2d')) profile.reset(); else view.fit();
  });
  const tabs = (three) => {
    document.body.classList.toggle('mode-2d', !three);
    $('tab3d').setAttribute('aria-pressed', String(three));
    $('tab2d').setAttribute('aria-pressed', String(!three));
    $('hint').textContent = three ? 'drag orbit · wheel zoom · shift-drag pan' : 'drag pan · wheel zoom';
    if (view) view.dirty = true;
    profile.dirty = true;
  };
  $('tab3d').addEventListener('click', () => tabs(true));
  $('tab2d').addEventListener('click', () => tabs(false));

  $('dlStep').addEventListener('click', () => {
    if (last) download(last.name + '.step', last.step(), 'application/step');
  });
  $('dlSvg').addEventListener('click', () => {
    if (last) download(last.name + '.svg', last.svg(), 'image/svg+xml');
  });
  $('dlStl').addEventListener('click', () => {
    if (last) download(last.name + '.stl', stlOf(last.mesh, last.name), 'model/stl');
  });
  $('dlLink').addEventListener('click', async (e) => {
    writeHash();
    try {
      await navigator.clipboard.writeText(location.href);
      e.target.textContent = 'copied';
    } catch { e.target.textContent = 'see url'; }
    setTimeout(() => { e.target.textContent = 'copy link'; }, 1400);
  });

  readHash();
  pushStateToUi();
  regenerate();
  if (view) view.fit();

  // handy from the console: window.gear.last() / .view / .regenerate()
  window.gear = { wasm: gear, view, profile, state, regenerate, last: () => last, stlOf };

  let t = performance.now();
  const loop = (now) => {
    const dt = Math.min((now - t) / 1000, 0.1); t = now;
    if (view) view.draw(dt);
    profile.draw();
    requestAnimationFrame(loop);
  };
  requestAnimationFrame(loop);
}

main().catch((e) => {
  document.getElementById('stats').textContent = 'failed: ' + e.message;
  console.error(e);
});
