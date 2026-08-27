//! Ear clipping triangulation of a polygon with holes (a Rust port of the
//! algorithm used by mapbox/earcut: hole elimination by bridges, ear clipping
//! accelerated with a z-order hash, plus the usual self-intersection cures).
//!
//! Input is a flat `[x0, y0, x1, y1, ...]` array; `hole_indices` gives the
//! first *vertex* index of each hole. Output is a flat list of vertex indices,
//! three per triangle, wound the same way as the outer ring.

const NIL: usize = usize::MAX;

struct Node {
    /// vertex index in the flat coordinate array
    i: usize,
    x: f64,
    y: f64,
    prev: usize,
    next: usize,
    z: i32,
    prev_z: usize,
    next_z: usize,
    steiner: bool,
}

type Arena = Vec<Node>;

fn insert_node(a: &mut Arena, i: usize, x: f64, y: f64, last: usize) -> usize {
    let id = a.len();
    a.push(Node {
        i,
        x,
        y,
        prev: NIL,
        next: NIL,
        z: 0,
        prev_z: NIL,
        next_z: NIL,
        steiner: false,
    });
    if last == NIL {
        a[id].prev = id;
        a[id].next = id;
    } else {
        let nx = a[last].next;
        a[id].next = nx;
        a[id].prev = last;
        a[nx].prev = id;
        a[last].next = id;
    }
    id
}

fn remove_node(a: &mut Arena, p: usize) {
    let (nx, pv) = (a[p].next, a[p].prev);
    a[nx].prev = pv;
    a[pv].next = nx;
    let (pz, nz) = (a[p].prev_z, a[p].next_z);
    if pz != NIL {
        a[pz].next_z = nz;
    }
    if nz != NIL {
        a[nz].prev_z = pz;
    }
}

/// Build a circular doubly linked list from a ring, in the requested winding.
fn linked_list(a: &mut Arena, data: &[f64], start: usize, end: usize, clockwise: bool) -> usize {
    let mut last = NIL;
    if clockwise == (signed_area(data, start, end) > 0.0) {
        let mut i = start;
        while i < end {
            last = insert_node(a, i, data[i], data[i + 1], last);
            i += 2;
        }
    } else {
        let mut i = end;
        while i >= start + 2 {
            i -= 2;
            last = insert_node(a, i, data[i], data[i + 1], last);
        }
    }
    if last != NIL && equals(a, last, a[last].next) {
        remove_node(a, last);
        last = a[last].next;
    }
    last
}

/// Drop collinear or duplicate points.
fn filter_points(a: &mut Arena, start: usize, end: usize) -> usize {
    if start == NIL {
        return start;
    }
    let end = if end == NIL { start } else { end };
    let mut p = start;
    let mut end = end;
    loop {
        let again;
        let nx = a[p].next;
        if !a[p].steiner && (equals(a, p, nx) || area(a, a[p].prev, p, nx) == 0.0) {
            remove_node(a, p);
            let pv = a[p].prev;
            p = pv;
            end = pv;
            if p == a[p].next {
                break;
            }
            again = true;
        } else {
            p = a[p].next;
            again = false;
        }
        if !again && p == end {
            break;
        }
    }
    end
}

#[allow(clippy::too_many_arguments)]
fn earcut_linked(
    a: &mut Arena,
    ear: usize,
    tri: &mut Vec<usize>,
    min_x: f64,
    min_y: f64,
    inv_size: f64,
    pass: u8,
) {
    let mut ear = ear;
    if ear == NIL {
        return;
    }
    if pass == 0 && inv_size > 0.0 {
        index_curve(a, ear, min_x, min_y, inv_size);
    }
    let mut stop = ear;
    while a[ear].prev != a[ear].next {
        let prev = a[ear].prev;
        let next = a[ear].next;
        let is = if inv_size > 0.0 {
            is_ear_hashed(a, ear, min_x, min_y, inv_size)
        } else {
            is_ear(a, ear)
        };
        if is {
            tri.push(a[prev].i / 2);
            tri.push(a[ear].i / 2);
            tri.push(a[next].i / 2);
            remove_node(a, ear);
            ear = a[next].next;
            stop = a[next].next;
            continue;
        }
        ear = next;
        if ear == stop {
            // no ear found: try the repair strategies, then give up
            if pass == 0 {
                let f = filter_points(a, ear, NIL);
                earcut_linked(a, f, tri, min_x, min_y, inv_size, 1);
            } else if pass == 1 {
                let f = filter_points(a, ear, NIL);
                let c = cure_local_intersections(a, f, tri);
                earcut_linked(a, c, tri, min_x, min_y, inv_size, 2);
            } else if pass == 2 {
                split_earcut(a, ear, tri, min_x, min_y, inv_size);
            }
            break;
        }
    }
}

fn is_ear(a: &Arena, ear: usize) -> bool {
    let (b, c) = (a[ear].prev, a[ear].next);
    if area(a, b, ear, c) >= 0.0 {
        return false; // reflex
    }
    let (ax, ay) = (a[b].x, a[b].y);
    let (bx, by) = (a[ear].x, a[ear].y);
    let (cx, cy) = (a[c].x, a[c].y);
    let x0 = ax.min(bx).min(cx);
    let y0 = ay.min(by).min(cy);
    let x1 = ax.max(bx).max(cx);
    let y1 = ay.max(by).max(cy);
    let mut p = a[c].next;
    while p != b {
        if a[p].x >= x0
            && a[p].x <= x1
            && a[p].y >= y0
            && a[p].y <= y1
            && point_in_triangle(ax, ay, bx, by, cx, cy, a[p].x, a[p].y)
            && area(a, a[p].prev, p, a[p].next) >= 0.0
        {
            return false;
        }
        p = a[p].next;
    }
    true
}

fn is_ear_hashed(a: &Arena, ear: usize, min_x: f64, min_y: f64, inv_size: f64) -> bool {
    let (b, c) = (a[ear].prev, a[ear].next);
    if area(a, b, ear, c) >= 0.0 {
        return false;
    }
    let (ax, ay) = (a[b].x, a[b].y);
    let (bx, by) = (a[ear].x, a[ear].y);
    let (cx, cy) = (a[c].x, a[c].y);
    let x0 = ax.min(bx).min(cx);
    let y0 = ay.min(by).min(cy);
    let x1 = ax.max(bx).max(cx);
    let y1 = ay.max(by).max(cy);
    let min_z = z_order(x0, y0, min_x, min_y, inv_size);
    let max_z = z_order(x1, y1, min_x, min_y, inv_size);

    let mut p = a[ear].prev_z;
    let mut n = a[ear].next_z;
    while p != NIL && a[p].z >= min_z && n != NIL && a[n].z <= max_z {
        if a[p].x >= x0
            && a[p].x <= x1
            && a[p].y >= y0
            && a[p].y <= y1
            && p != b
            && p != c
            && point_in_triangle(ax, ay, bx, by, cx, cy, a[p].x, a[p].y)
            && area(a, a[p].prev, p, a[p].next) >= 0.0
        {
            return false;
        }
        p = a[p].prev_z;
        if a[n].x >= x0
            && a[n].x <= x1
            && a[n].y >= y0
            && a[n].y <= y1
            && n != b
            && n != c
            && point_in_triangle(ax, ay, bx, by, cx, cy, a[n].x, a[n].y)
            && area(a, a[n].prev, n, a[n].next) >= 0.0
        {
            return false;
        }
        n = a[n].next_z;
    }
    while p != NIL && a[p].z >= min_z {
        if a[p].x >= x0
            && a[p].x <= x1
            && a[p].y >= y0
            && a[p].y <= y1
            && p != b
            && p != c
            && point_in_triangle(ax, ay, bx, by, cx, cy, a[p].x, a[p].y)
            && area(a, a[p].prev, p, a[p].next) >= 0.0
        {
            return false;
        }
        p = a[p].prev_z;
    }
    while n != NIL && a[n].z <= max_z {
        if a[n].x >= x0
            && a[n].x <= x1
            && a[n].y >= y0
            && a[n].y <= y1
            && n != b
            && n != c
            && point_in_triangle(ax, ay, bx, by, cx, cy, a[n].x, a[n].y)
            && area(a, a[n].prev, n, a[n].next) >= 0.0
        {
            return false;
        }
        n = a[n].next_z;
    }
    true
}

fn cure_local_intersections(a: &mut Arena, start: usize, tri: &mut Vec<usize>) -> usize {
    let mut p = start;
    let mut start = start;
    loop {
        let b = a[p].prev;
        let n = a[p].next;
        let d = a[n].next;
        if !equals(a, b, d) && intersects(a, b, p, n, d) && locally_inside(a, b, d) && locally_inside(a, d, b) {
            tri.push(a[b].i / 2);
            tri.push(a[p].i / 2);
            tri.push(a[d].i / 2);
            remove_node(a, p);
            remove_node(a, n);
            p = d;
            start = d;
        }
        p = a[p].next;
        if p == start {
            break;
        }
    }
    filter_points(a, p, NIL)
}

fn split_earcut(
    a: &mut Arena,
    start: usize,
    tri: &mut Vec<usize>,
    min_x: f64,
    min_y: f64,
    inv_size: f64,
) {
    let mut p = start;
    loop {
        let mut q = a[a[p].next].next;
        while q != a[p].prev {
            if a[p].i != a[q].i && is_valid_diagonal(a, p, q) {
                let mut c = split_polygon(a, p, q);
                let fp = filter_points(a, p, a[p].next);
                let fc = filter_points(a, c, a[c].next);
                c = fc;
                earcut_linked(a, fp, tri, min_x, min_y, inv_size, 0);
                earcut_linked(a, c, tri, min_x, min_y, inv_size, 0);
                return;
            }
            q = a[q].next;
        }
        p = a[p].next;
        if p == start {
            break;
        }
    }
}

fn eliminate_holes(
    a: &mut Arena,
    data: &[f64],
    hole_indices: &[usize],
    outer_node: usize,
) -> usize {
    let mut queue: Vec<usize> = Vec::new();
    let len = hole_indices.len();
    for i in 0..len {
        let start = hole_indices[i] * 2;
        let end = if i < len - 1 { hole_indices[i + 1] * 2 } else { data.len() };
        let list = linked_list(a, data, start, end, false);
        if list != NIL {
            if list == a[list].next {
                a[list].steiner = true;
            }
            queue.push(get_leftmost(a, list));
        }
    }
    queue.sort_by(|&p, &q| a[p].x.partial_cmp(&a[q].x).unwrap_or(std::cmp::Ordering::Equal));
    let mut outer = outer_node;
    for &h in &queue {
        outer = eliminate_hole(a, h, outer);
    }
    outer
}

fn eliminate_hole(a: &mut Arena, hole: usize, outer_node: usize) -> usize {
    let bridge = find_hole_bridge(a, hole, outer_node);
    if bridge == NIL {
        return outer_node;
    }
    let bridge_reverse = split_polygon(a, bridge, hole);
    filter_points(a, bridge_reverse, a[bridge_reverse].next);
    filter_points(a, bridge, a[bridge].next)
}

/// David Eberly's algorithm: find a bridge from the hole's leftmost point to
/// the outer ring.
fn find_hole_bridge(a: &Arena, hole: usize, outer_node: usize) -> usize {
    let hx = a[hole].x;
    let hy = a[hole].y;
    let mut qx = f64::NEG_INFINITY;
    let mut m = NIL;
    let mut p = outer_node;
    loop {
        let n = a[p].next;
        if hy <= a[p].y && hy >= a[n].y && a[n].y != a[p].y {
            let x = a[p].x + (hy - a[p].y) * (a[n].x - a[p].x) / (a[n].y - a[p].y);
            if x <= hx && x > qx {
                qx = x;
                m = if a[p].x < a[n].x { p } else { n };
                if x == hx {
                    return m;
                }
            }
        }
        p = n;
        if p == outer_node {
            break;
        }
    }
    if m == NIL {
        return NIL;
    }
    // look for a reflex vertex inside the (hx,hy)-(qx,hy)-(mx,my) triangle
    let stop = m;
    let mx = a[m].x;
    let my = a[m].y;
    let mut tan_min = f64::INFINITY;
    let mut m = m;
    let mut p = m;
    loop {
        let px = a[p].x;
        let py = a[p].y;
        if hx >= px
            && px >= mx
            && hx != px
            && point_in_triangle(
                if hy < my { hx } else { qx },
                hy,
                mx,
                my,
                if hy < my { qx } else { hx },
                hy,
                px,
                py,
            )
        {
            let tan = (hy - py).abs() / (hx - px);
            if locally_inside(a, p, hole)
                && (tan < tan_min
                    || (tan == tan_min
                        && (px > a[m].x || (px == a[m].x && sector_contains_sector(a, m, p)))))
            {
                m = p;
                tan_min = tan;
            }
        }
        p = a[p].next;
        if p == stop {
            break;
        }
    }
    m
}

fn sector_contains_sector(a: &Arena, m: usize, p: usize) -> bool {
    area(a, a[m].prev, m, a[p].prev) < 0.0 && area(a, a[p].next, m, a[m].next) < 0.0
}

fn index_curve(a: &mut Arena, start: usize, min_x: f64, min_y: f64, inv_size: f64) {
    let mut p = start;
    loop {
        if a[p].z == 0 {
            a[p].z = z_order(a[p].x, a[p].y, min_x, min_y, inv_size);
        }
        a[p].prev_z = a[p].prev;
        a[p].next_z = a[p].next;
        p = a[p].next;
        if p == start {
            break;
        }
    }
    let pz = a[p].prev_z;
    a[pz].next_z = NIL;
    a[p].prev_z = NIL;
    sort_linked(a, p);
}

/// Merge sort on the z-order linked list.
fn sort_linked(a: &mut Arena, list: usize) -> usize {
    let mut list = list;
    let mut in_size = 1usize;
    loop {
        let mut p = list;
        list = NIL;
        let mut tail = NIL;
        let mut num_merges = 0;
        while p != NIL {
            num_merges += 1;
            let mut q = p;
            let mut p_size = 0usize;
            for _ in 0..in_size {
                p_size += 1;
                q = a[q].next_z;
                if q == NIL {
                    break;
                }
            }
            let mut q_size = in_size;
            while p_size > 0 || (q_size > 0 && q != NIL) {
                let e;
                if p_size == 0 {
                    e = q;
                    q = a[q].next_z;
                    q_size -= 1;
                } else if q_size == 0 || q == NIL {
                    e = p;
                    p = a[p].next_z;
                    p_size -= 1;
                } else if a[p].z <= a[q].z {
                    e = p;
                    p = a[p].next_z;
                    p_size -= 1;
                } else {
                    e = q;
                    q = a[q].next_z;
                    q_size -= 1;
                }
                if tail != NIL {
                    a[tail].next_z = e;
                } else {
                    list = e;
                }
                a[e].prev_z = tail;
                tail = e;
            }
            p = q;
        }
        a[tail].next_z = NIL;
        if num_merges <= 1 {
            return list;
        }
        in_size *= 2;
    }
}

/// 32 bit Morton code of a point in the unit square scaled to 15 bits.
fn z_order(x: f64, y: f64, min_x: f64, min_y: f64, inv_size: f64) -> i32 {
    let mut x = (((x - min_x) * inv_size) as i64).clamp(0, 32767) as i32;
    let mut y = (((y - min_y) * inv_size) as i64).clamp(0, 32767) as i32;
    x = (x | (x << 8)) & 0x00FF00FF;
    x = (x | (x << 4)) & 0x0F0F0F0F;
    x = (x | (x << 2)) & 0x33333333;
    x = (x | (x << 1)) & 0x55555555;
    y = (y | (y << 8)) & 0x00FF00FF;
    y = (y | (y << 4)) & 0x0F0F0F0F;
    y = (y | (y << 2)) & 0x33333333;
    y = (y | (y << 1)) & 0x55555555;
    x | (y << 1)
}

fn get_leftmost(a: &Arena, start: usize) -> usize {
    let mut p = start;
    let mut leftmost = start;
    loop {
        if a[p].x < a[leftmost].x || (a[p].x == a[leftmost].x && a[p].y < a[leftmost].y) {
            leftmost = p;
        }
        p = a[p].next;
        if p == start {
            break;
        }
    }
    leftmost
}

#[allow(clippy::too_many_arguments)]
fn point_in_triangle(ax: f64, ay: f64, bx: f64, by: f64, cx: f64, cy: f64, px: f64, py: f64) -> bool {
    (cx - px) * (ay - py) >= (ax - px) * (cy - py)
        && (ax - px) * (by - py) >= (bx - px) * (ay - py)
        && (bx - px) * (cy - py) >= (cx - px) * (by - py)
}

fn is_valid_diagonal(a: &Arena, p: usize, q: usize) -> bool {
    a[a[p].next].i != a[q].i
        && a[a[p].prev].i != a[q].i
        && !intersects_polygon(a, p, q)
        && ((locally_inside(a, p, q)
            && locally_inside(a, q, p)
            && middle_inside(a, p, q)
            && (area(a, a[p].prev, p, a[q].prev) != 0.0 || area(a, p, a[q].prev, q) != 0.0))
            || (equals(a, p, q)
                && area(a, a[p].prev, p, a[p].next) > 0.0
                && area(a, a[q].prev, q, a[q].next) > 0.0))
}

/// Twice the signed area of the triangle p-q-r.
fn area(a: &Arena, p: usize, q: usize, r: usize) -> f64 {
    (a[q].y - a[p].y) * (a[r].x - a[q].x) - (a[q].x - a[p].x) * (a[r].y - a[q].y)
}

fn equals(a: &Arena, p: usize, q: usize) -> bool {
    a[p].x == a[q].x && a[p].y == a[q].y
}

fn sign(v: f64) -> i32 {
    if v > 0.0 {
        1
    } else if v < 0.0 {
        -1
    } else {
        0
    }
}

fn intersects(a: &Arena, p1: usize, q1: usize, p2: usize, q2: usize) -> bool {
    let o1 = sign(area(a, p1, q1, p2));
    let o2 = sign(area(a, p1, q1, q2));
    let o3 = sign(area(a, p2, q2, p1));
    let o4 = sign(area(a, p2, q2, q1));
    if o1 != o2 && o3 != o4 {
        return true;
    }
    if o1 == 0 && on_segment(a, p1, p2, q1) {
        return true;
    }
    if o2 == 0 && on_segment(a, p1, q2, q1) {
        return true;
    }
    if o3 == 0 && on_segment(a, p2, p1, q2) {
        return true;
    }
    if o4 == 0 && on_segment(a, p2, q1, q2) {
        return true;
    }
    false
}

fn on_segment(a: &Arena, p: usize, q: usize, r: usize) -> bool {
    a[q].x <= a[p].x.max(a[r].x)
        && a[q].x >= a[p].x.min(a[r].x)
        && a[q].y <= a[p].y.max(a[r].y)
        && a[q].y >= a[p].y.min(a[r].y)
}

fn intersects_polygon(a: &Arena, p: usize, q: usize) -> bool {
    let mut n = p;
    loop {
        let nx = a[n].next;
        if a[n].i != a[p].i
            && a[nx].i != a[p].i
            && a[n].i != a[q].i
            && a[nx].i != a[q].i
            && intersects(a, n, nx, p, q)
        {
            return true;
        }
        n = nx;
        if n == p {
            break;
        }
    }
    false
}

fn locally_inside(a: &Arena, p: usize, q: usize) -> bool {
    let (pv, nx) = (a[p].prev, a[p].next);
    if area(a, pv, p, nx) < 0.0 {
        area(a, p, q, nx) >= 0.0 && area(a, p, pv, q) >= 0.0
    } else {
        area(a, p, q, pv) < 0.0 || area(a, p, nx, q) < 0.0
    }
}

fn middle_inside(a: &Arena, p: usize, q: usize) -> bool {
    let mut n = p;
    let mut inside = false;
    let px = (a[p].x + a[q].x) / 2.0;
    let py = (a[p].y + a[q].y) / 2.0;
    loop {
        let nx = a[n].next;
        if ((a[n].y > py) != (a[nx].y > py))
            && a[nx].y != a[n].y
            && (px < (a[nx].x - a[n].x) * (py - a[n].y) / (a[nx].y - a[n].y) + a[n].x)
        {
            inside = !inside;
        }
        n = nx;
        if n == p {
            break;
        }
    }
    inside
}

/// Cut the polygon in two along the diagonal p-q; returns the new node.
fn split_polygon(a: &mut Arena, p: usize, q: usize) -> usize {
    let p2 = a.len();
    a.push(Node {
        i: a[p].i,
        x: a[p].x,
        y: a[p].y,
        prev: NIL,
        next: NIL,
        z: 0,
        prev_z: NIL,
        next_z: NIL,
        steiner: false,
    });
    let q2 = a.len();
    a.push(Node {
        i: a[q].i,
        x: a[q].x,
        y: a[q].y,
        prev: NIL,
        next: NIL,
        z: 0,
        prev_z: NIL,
        next_z: NIL,
        steiner: false,
    });
    let pn = a[p].next;
    let qp = a[q].prev;
    a[p].next = q;
    a[q].prev = p;
    a[p2].next = pn;
    a[pn].prev = p2;
    a[q2].next = p2;
    a[p2].prev = q2;
    a[qp].next = q2;
    a[q2].prev = qp;
    q2
}

fn signed_area(data: &[f64], start: usize, end: usize) -> f64 {
    let mut sum = 0.0;
    let mut j = if end >= 2 { end - 2 } else { start };
    let mut i = start;
    while i < end {
        sum += (data[j] - data[i]) * (data[i + 1] + data[j + 1]);
        j = i;
        i += 2;
    }
    sum
}

/// Triangulate a polygon with holes. `data` is `[x, y, x, y, ...]`,
/// `hole_indices` holds the first vertex index of every hole.
pub fn earcut(data: &[f64], hole_indices: &[usize]) -> Vec<usize> {
    let mut tri: Vec<usize> = Vec::new();
    if data.len() < 6 {
        return tri;
    }
    let has_holes = !hole_indices.is_empty();
    let outer_len = if has_holes { hole_indices[0] * 2 } else { data.len() };
    let mut a: Arena = Vec::with_capacity(data.len() / 2 + 32);
    let mut outer_node = linked_list(&mut a, data, 0, outer_len, true);
    if outer_node == NIL || a[outer_node].next == a[outer_node].prev {
        return tri;
    }
    if has_holes {
        outer_node = eliminate_holes(&mut a, data, hole_indices, outer_node);
    }

    let (mut min_x, mut min_y, mut inv_size) = (0.0, 0.0, 0.0);
    if data.len() > 160 {
        min_x = data[0];
        min_y = data[1];
        let mut max_x = data[0];
        let mut max_y = data[1];
        let mut i = 2;
        while i < outer_len {
            min_x = min_x.min(data[i]);
            max_x = max_x.max(data[i]);
            min_y = min_y.min(data[i + 1]);
            max_y = max_y.max(data[i + 1]);
            i += 2;
        }
        inv_size = (max_x - min_x).max(max_y - min_y);
        inv_size = if inv_size != 0.0 { 32767.0 / inv_size } else { 0.0 };
    }
    earcut_linked(&mut a, outer_node, &mut tri, min_x, min_y, inv_size, 0);
    tri
}
