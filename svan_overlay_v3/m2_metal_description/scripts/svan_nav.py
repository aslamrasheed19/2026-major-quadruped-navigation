"""svan_nav.py -- ROS-free obstacle avoidance for the Svan M2 (pure Python + NumPy).

    Grid         occupancy grid (persistent obstacle cells) + distance-to-obstacle field
    plan_path    A* on the grid with a hard robot-radius inflation, a soft clearance cost and line-of-sight smoothing
    PathFollower pure-pursuit style follower: turns a planned polyline into (vx, wz) for SvanController.command()
    ScanMapper   turns 2-D range scans (lidar) + the robot pose into obstacle cells (log-odds, so one stray hit is forgotten)
    ForwardGuard slows / stops the robot when the raw scan shows something right in front of it
    Navigator    ties the above together: modes "known" (obstacles given beforehand) and "sensed" (obstacles only from scans)

Everything here is plain geometry, so the same file is tested offline (MuJoCo, see tools/) and imported by the ROS node.
World frame: x forward at start, y left, yaw counter-clockwise.  All lengths in metres.
"""
import heapq
import math

import numpy as np


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# ======================================================================================================
class Grid:
    """Square-cell occupancy grid.  occ[iy, ix] is True for a cell that contains an obstacle."""

    def __init__(self, xmin=-1.5, ymin=-1.5, xmax=3.5, ymax=3.5, res=0.05):
        self.res, self.x0, self.y0 = res, xmin, ymin
        self.nx, self.ny = int(math.ceil((xmax - xmin) / res)), int(math.ceil((ymax - ymin) / res))
        self.occ = np.zeros((self.ny, self.nx), bool)
        self.logodds = np.zeros((self.ny, self.nx), np.float32)
        self.version = 0                       # bumped whenever occ changes (the planner re-computes its distance field)
        self._dist = None; self._dist_version = -1

    # ---- coordinates ------------------------------------------------------------------------
    def cell(self, x, y):
        return int(math.floor((x - self.x0) / self.res)), int(math.floor((y - self.y0) / self.res))

    def centre(self, ix, iy):
        return self.x0 + (ix + 0.5) * self.res, self.y0 + (iy + 0.5) * self.res

    def inside(self, ix, iy):
        return 0 <= ix < self.nx and 0 <= iy < self.ny

    # ---- known obstacles --------------------------------------------------------------------
    def _mesh(self):
        xs = self.x0 + (np.arange(self.nx) + 0.5) * self.res
        ys = self.y0 + (np.arange(self.ny) + 0.5) * self.res
        return np.meshgrid(xs, ys)

    def add_circle(self, cx, cy, r):
        X, Y = self._mesh(); self.occ |= (X - cx) ** 2 + (Y - cy) ** 2 <= r * r; self.version += 1

    def add_box(self, cx, cy, w, h, yaw=0.0):
        """w along the box's own x axis, h along its y axis; yaw rotates it about its centre."""
        X, Y = self._mesh(); c, s = math.cos(yaw), math.sin(yaw)
        u = (X - cx) * c + (Y - cy) * s; v = -(X - cx) * s + (Y - cy) * c
        self.occ |= (np.abs(u) <= w / 2) & (np.abs(v) <= h / 2); self.version += 1

    # ---- distance to the nearest obstacle (capped) -------------------------------------------
    def dist(self, cap=0.75):
        if self._dist is not None and self._dist_version == self.version:
            return self._dist
        res, k = self.res, int(math.ceil(cap / self.res))
        d = np.full((self.ny, self.nx), cap, np.float32)
        if self.occ.any():
            for dy in range(-k, k + 1):
                for dx in range(-k, k + 1):
                    dd = math.hypot(dx, dy) * res
                    if dd >= cap:
                        continue
                    y0, y1 = max(0, -dy), min(self.ny, self.ny - dy); x0, x1 = max(0, -dx), min(self.nx, self.nx - dx)
                    src = self.occ[y0 + dy:y1 + dy, x0 + dx:x1 + dx]; dst = d[y0:y1, x0:x1]
                    m = src & (dst > dd); dst[m] = dd
        self._dist, self._dist_version = d, self.version
        return d

    def clearance(self, x, y):
        ix, iy = self.cell(x, y)
        return float(self.dist()[iy, ix]) if self.inside(ix, iy) else 0.0


# ======================================================================================================
def _nearest_free(dist, grid, cell, r_hard, max_cells=14):
    """Closest cell (by ring search) whose clearance is >= r_hard.  None if nothing within max_cells."""
    ix, iy = cell
    if grid.inside(ix, iy) and dist[iy, ix] >= r_hard:
        return cell
    best, bd = None, 1e9
    for dy in range(-max_cells, max_cells + 1):
        for dx in range(-max_cells, max_cells + 1):
            jx, jy = ix + dx, iy + dy
            if grid.inside(jx, jy) and dist[jy, jx] >= r_hard:
                d = dx * dx + dy * dy
                if d < bd:
                    best, bd = (jx, jy), d
    return best


def segment_clear(grid, a, b, thr):
    """True if every point on the segment a->b has clearance >= thr."""
    dist = grid.dist(); n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (grid.res * 0.5)))
    for i in range(n + 1):
        t = i / n; ix, iy = grid.cell(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
        if not grid.inside(ix, iy) or dist[iy, ix] < thr:
            return False
    return True


def plan_path(grid, start, goal, r_hard=0.38, r_soft=0.65, soft_w=1.0, smooth_slack=0.03):
    """A* from start to goal (world xy).  Returns (points, info) with points = [(x, y), ...] or None.

    Cells with clearance < r_hard are blocked.  Between r_hard and r_soft a cost penalty pushes the path away from
    obstacles.  A start or goal that sits inside the blocked band is moved to the nearest free cell (info says so)."""
    dist = grid.dist(); res = grid.res; nx, ny = grid.nx, grid.ny
    info = {"start_moved": False, "goal_moved": False}
    sc, gc = grid.cell(*start), grid.cell(*goal)
    if not grid.inside(*sc) or not grid.inside(*gc):
        return None, dict(info, reason="start or goal outside the map")
    s_free = _nearest_free(dist, grid, sc, r_hard); g_free = _nearest_free(dist, grid, gc, r_hard)
    if s_free is None or g_free is None:
        return None, dict(info, reason="no free cell near start or goal")
    info["start_moved"] = s_free != sc; info["goal_moved"] = g_free != gc
    flat_cost = 1.0 + soft_w * np.clip((r_soft - dist) / max(r_soft - r_hard, 1e-6), 0.0, 1.0)
    blocked = dist < r_hard
    s_i, g_i = s_free[1] * nx + s_free[0], g_free[1] * nx + g_free[0]
    gx, gy = g_free
    INF = 1e18
    g = np.full(nx * ny, INF); parent = np.full(nx * ny, -1, np.int64); closed = np.zeros(nx * ny, bool)
    g[s_i] = 0.0
    heap = [(0.0, s_i)]
    nbr = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0), (-1, -1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (1, 1, 1.4142)]
    bl = blocked.ravel(); fc = flat_cost.ravel()
    while heap:
        _, i = heapq.heappop(heap)
        if closed[i]:
            continue
        closed[i] = True
        if i == g_i:
            break
        cy, cx = divmod(i, nx)
        for dx, dy, sl in nbr:
            jx, jy = cx + dx, cy + dy
            if jx < 0 or jy < 0 or jx >= nx or jy >= ny:
                continue
            j = jy * nx + jx
            if closed[j] or bl[j]:
                continue
            if dx and dy and (bl[cy * nx + jx] or bl[jy * nx + cx]):          # no corner cutting
                continue
            ng = g[i] + sl * res * fc[j]
            if ng < g[j]:
                g[j] = ng; parent[j] = i
                h = res * (max(abs(gx - jx), abs(gy - jy)) + 0.4142 * min(abs(gx - jx), abs(gy - jy)))
                heapq.heappush(heap, (ng + h, j))
    if parent[g_i] < 0 and s_i != g_i:
        return None, dict(info, reason="no path")
    cells = []; i = g_i
    while i != -1:
        cells.append(divmod(i, nx)[::-1]); i = parent[i]
    cells.reverse()
    pts = [tuple(map(float, grid.centre(int(ix), int(iy)))) for ix, iy in cells]
    pts[0] = tuple(map(float, start)) if not info["start_moved"] else pts[0]
    if info["start_moved"]:
        pts.insert(0, tuple(map(float, start)))
    if info["goal_moved"]:
        info["goal_used"] = pts[-1]
    elif len(pts) == 1:
        pts.append(tuple(map(float, goal)))
    else:
        pts[-1] = tuple(map(float, goal))
    # line-of-sight smoothing; keep a little extra clearance so a follower cutting corners stays outside the hard band
    thr = r_hard + smooth_slack
    out = [pts[0]]; i = 0
    while i < len(pts) - 1:
        j = len(pts) - 1
        while j > i + 1 and not segment_clear(grid, pts[i], pts[j], thr if i > 0 or not info["start_moved"] else r_hard * 0.5):
            j -= 1
        out.append(pts[j]); i = j
    length = sum(math.hypot(out[k + 1][0] - out[k][0], out[k + 1][1] - out[k][1]) for k in range(len(out) - 1))
    info["length"] = length
    return out, info


def path_clear(grid, pts, r_hard):
    """Is the polyline still free of (inflated) obstacles on the current grid?"""
    for k in range(len(pts) - 1):
        if not segment_clear(grid, pts[k], pts[k + 1], r_hard):
            return False
    return True


# ======================================================================================================
class PathFollower:
    """Follow a polyline: aim at the point `lookahead` metres ahead of the closest point, same speed/turn rules as
    svan_core.WaypointFollower (turn on the spot above `turn_thresh`, slow down near the end)."""

    def __init__(self, pts, v_max=0.25, w_max=0.8, k_head=1.5, reach=0.08, turn_thresh=0.6, slow_dist=0.4, lookahead=0.30):
        self.P = [tuple(map(float, p)) for p in pts]
        self.v_max, self.w_max, self.k, self.reach, self.turn, self.slow, self.L = v_max, w_max, k_head, reach, turn_thresh, slow_dist, lookahead
        self.S = [0.0]
        for a, b in zip(self.P[:-1], self.P[1:]):
            self.S.append(self.S[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self.total = self.S[-1]; self.seg = 0; self.s = 0.0
        self.cross = 0.0; self.done = len(self.P) < 2

    def _point_at(self, s):
        s = min(max(s, 0.0), self.total)
        for i in range(len(self.P) - 1):
            if s <= self.S[i + 1] or i == len(self.P) - 2:
                seg = self.S[i + 1] - self.S[i]; t = 0.0 if seg < 1e-9 else (s - self.S[i]) / seg
                return (self.P[i][0] + t * (self.P[i + 1][0] - self.P[i][0]), self.P[i][1] + t * (self.P[i + 1][1] - self.P[i][1]))
        return self.P[-1]

    def update(self, x, y, yaw):
        """-> (vx, wz).  Sets self.done and self.cross (distance from the path)."""
        if self.done:
            return 0.0, 0.0
        best = None
        for i in range(max(0, self.seg - 1), min(len(self.P) - 1, self.seg + 4)):
            ax, ay = self.P[i]; bx, by = self.P[i + 1]; dx, dy = bx - ax, by - ay; ll = dx * dx + dy * dy
            t = 0.0 if ll < 1e-12 else min(1.0, max(0.0, ((x - ax) * dx + (y - ay) * dy) / ll))
            qx, qy = ax + t * dx, ay + t * dy; d = math.hypot(x - qx, y - qy)
            if best is None or d < best[0]:
                best = (d, i, self.S[i] + t * math.sqrt(ll))
        self.cross, self.seg = best[0], best[1]
        self.s = max(self.s, best[2])
        end = self.P[-1]; dist_end = math.hypot(end[0] - x, end[1] - y)
        if dist_end < self.reach:
            self.done = True; return 0.0, 0.0
        tx, ty = self._point_at(self.s + self.L) if (self.total - self.s) > self.L else end
        err = wrap(math.atan2(ty - y, tx - x) - yaw)
        wz = max(-self.w_max, min(self.w_max, self.k * err))
        if abs(err) > self.turn:
            return 0.0, wz
        vx = self.v_max * min(1.0, dist_end / self.slow) * max(0.0, math.cos(err))
        return max(vx, 0.08), wz


# ======================================================================================================
class ScanMapper:
    """Log-odds mapping from a 2-D scan.  Cells along a ray become 'more free', the cell at a measured range
    becomes 'more occupied'.  Rays with no return (inf / beyond r_max) only clear cells.  A cell counts as an
    obstacle once its log-odds exceed `occ_thresh`, i.e. after about two hits without clear rays in between."""

    def __init__(self, grid, sensor_xy=(0.05, 0.0), r_min=0.20, r_max=4.0, l_hit=0.85, l_miss=0.25, occ_thresh=1.2, l_min=-2.0, l_max=4.0, sensor_h=0.40):
        self.g, self.sxy, self.r_min, self.r_max, self.h = grid, sensor_xy, r_min, r_max, sensor_h
        self.l_hit, self.l_miss, self.thr, self.lo, self.hi = l_hit, l_miss, occ_thresh, l_min, l_max
        self.scans = 0

    def update(self, x, y, yaw, ranges, angle_min, angle_inc, tilt=0.0):
        """tilt = body roll/pitch magnitude [rad].  A level beam of a tilted body hits the FLOOR at about h / sin(tilt), so
        echoes beyond 85 % of that range are not trusted as obstacles (the effective range shrinks as the body tilts)."""
        g = self.g
        r = np.asarray(ranges, float); n = r.size
        r_max = min(self.r_max, 0.85 * self.h / max(math.sin(abs(tilt)), 1e-3))
        ang = yaw + angle_min + angle_inc * np.arange(n)
        c, s = math.cos(yaw), math.sin(yaw)
        ox = x + c * self.sxy[0] - s * self.sxy[1]; oy = y + s * self.sxy[0] + c * self.sxy[1]
        valid = np.isfinite(r) & (r >= self.r_min)
        hit = valid & (r < r_max - 0.05)
        noreturn = np.isposinf(r)                                            # no echo: everything up to r_max is free
        rr = np.where(valid, np.minimum(r, r_max), np.where(noreturn, r_max, 0.0))
        step = g.res * 0.8
        t = np.arange(0.0, r_max, step)                                 # (M,)
        keep = t[None, :] < (rr[:, None] - g.res)                            # free samples strictly before the hit cell
        px = ox + np.cos(ang)[:, None] * t[None, :]; py = oy + np.sin(ang)[:, None] * t[None, :]
        ix = np.floor((px - g.x0) / g.res).astype(np.int64); iy = np.floor((py - g.y0) / g.res).astype(np.int64)
        ok = keep & (ix >= 0) & (ix < g.nx) & (iy >= 0) & (iy < g.ny)
        free_idx = np.unique(iy[ok] * g.nx + ix[ok])
        hx = ox + np.cos(ang[hit]) * r[hit]; hy = oy + np.sin(ang[hit]) * r[hit]
        hix = np.floor((hx - g.x0) / g.res).astype(np.int64); hiy = np.floor((hy - g.y0) / g.res).astype(np.int64)
        hok = (hix >= 0) & (hix < g.nx) & (hiy >= 0) & (hiy < g.ny)
        hit_idx = np.unique(hiy[hok] * g.nx + hix[hok])
        free_idx = np.setdiff1d(free_idx, hit_idx)                           # a cell that holds an echo is not also 'free'
        lo = g.logodds.reshape(-1)
        lo[free_idx] -= self.l_miss
        lo[hit_idx] += self.l_hit
        np.clip(lo, self.lo, self.hi, out=lo)
        new_occ = lo.reshape(g.ny, g.nx) > self.thr
        if not np.array_equal(new_occ, g.occ):
            g.occ = new_occ; g.version += 1
        self.scans += 1


# ======================================================================================================
class ForwardGuard:
    """Speed scale (0..1) from the raw scan: looks at returns inside a corridor straight ahead of the robot."""

    def __init__(self, half_width=0.30, look=1.0, front=0.26, d_stop=0.10, d_slow=0.45, sensor_x=0.05):
        self.hw, self.look, self.front, self.d_stop, self.d_slow = half_width, look, front, d_stop, d_slow
        self.clear = 9.9

    def update(self, ranges, angle_min, angle_inc, r_min=0.20):
        r = np.asarray(ranges, float); ang = angle_min + angle_inc * np.arange(r.size)
        ok = np.isfinite(r) & (r >= r_min)
        xr = r * np.cos(ang); yr = r * np.sin(ang)
        m = ok & (xr > 0.0) & (xr < self.look) & (np.abs(yr) < self.hw)
        self.clear = float(np.min(xr[m]) - self.front) if m.any() else 9.9
        return max(0.0, min(1.0, (self.clear - self.d_stop) / (self.d_slow - self.d_stop)))


# ======================================================================================================
class Navigator:
    """Mission manager.  Visits `waypoints` in order; for each one plans a path on the grid and follows it.

    mode "known":   the grid is filled once from `circles` [(x, y, r)] and `boxes` [(x, y, w, h)].
    mode "sensed":  the grid starts empty and is built from scans (update(..., scan=(ranges, angle_min, angle_inc))).
    update() returns (vx, wz, status) where status is one of "run", "done", "no_path"."""

    def __init__(self, mode, waypoints, circles=(), boxes=(), bounds=(-1.5, -1.5, 3.5, 3.5), res=0.05, robot_radius=0.32, margin=0.10,
                 v_max=0.25, w_max=0.8, k_head=1.5, reach=0.08, lookahead=0.30, sensor_xy=(0.05, 0.0), r_max=4.0, tilt_limit=0.20, guard=True,
                 replan_every=4.0, min_replan_gap=0.6):
        assert mode in ("known", "sensed")
        self.mode = mode; self.wps = [tuple(map(float, w)) for w in waypoints]; self.gi = 0
        self.grid = Grid(*bounds, res=res)
        self.r_hard = robot_radius + margin; self.r_soft = self.r_hard + 0.20
        self.r_block = robot_radius                               # hysteresis: plan with r_hard, call a path 'blocked' only below r_block
        self.fp = dict(v_max=v_max, w_max=w_max, k_head=k_head, reach=reach, lookahead=lookahead)
        self.reach = reach; self.tilt_limit = tilt_limit; self.replan_every = replan_every; self.min_gap = min_replan_gap
        if mode == "known":
            for (cx, cy, r) in circles:
                self.grid.add_circle(cx, cy, r)
            for (cx, cy, w, h) in boxes:
                self.grid.add_box(cx, cy, w, h)
            self.mapper = None; self.guard = None
        else:
            self.mapper = ScanMapper(self.grid, sensor_xy=sensor_xy, r_max=r_max)
            self.guard = ForwardGuard(sensor_x=sensor_xy[0]) if guard else None
        self.follower = None; self.path = None; self.t_plan = -1e9; self.t_progress = 0.0; self.s_best = 0.0
        self.replans = 0; self.last_info = {}; self.status = "run"; self.log = []
        self.scale = 1.0

    # ---- helpers ------------------------------------------------------------------------------
    def goal(self):
        return self.wps[self.gi] if self.gi < len(self.wps) else None

    def _plan(self, t, x, y, why):
        pts, info = plan_path(self.grid, (x, y), self.goal(), r_hard=self.r_hard, r_soft=self.r_soft)
        self.t_plan = t; self.last_info = info
        if pts is None or len(pts) < 2:
            self.path = None; self.follower = None; self.status = "no_path"
            self.log.append((t, "no path to waypoint %d (%s)" % (self.gi, info.get("reason", "?")))); return False
        self.path = pts; self.follower = PathFollower(pts, **self.fp); self.status = "run"; self.replans += 1; self.s_best = 0.0; self.t_progress = t
        self.log.append((t, "plan %d: %s -> waypoint %d, %d points, %.2f m%s" % (self.replans, why, self.gi, len(pts), info["length"], " (goal moved)" if info.get("goal_moved") else "")))
        return True

    # ---- main entry ----------------------------------------------------------------------------
    def update(self, t, x, y, yaw, tilt=0.0, scan=None, scan_pose=None):
        """scan = (ranges, angle_min, angle_increment) or None; scan_pose = (x, y, yaw, tilt) of the robot AT THE SCAN STAMP
        (a few ms older than x, y, yaw -- it matters when the robot turns).  Default: the current pose."""
        if self.mapper is not None and scan is not None:
            sx, sy, syaw, stilt = scan_pose if scan_pose is not None else (x, y, yaw, tilt)
            if stilt < self.tilt_limit:
                self.mapper.update(sx, sy, syaw, *scan, tilt=stilt)
            if self.guard is not None:
                self.scale = self.guard.update(*scan)                       # held until the next scan arrives
        scale = self.scale
        g = self.goal()
        if g is None:
            self.status = "done"; return 0.0, 0.0, "done"
        if math.hypot(g[0] - x, g[1] - y) < self.reach:
            self.log.append((t, "waypoint %d reached at (%.2f, %.2f)" % (self.gi, x, y)))
            self.gi += 1; self.path = None; self.follower = None
            g = self.goal()
            if g is None:
                self.status = "done"; return 0.0, 0.0, "done"
        why = None
        if self.path is None:
            why = "first plan" if self.replans == 0 else "next waypoint"
            if self.status == "no_path" and t - self.t_plan < 1.0:
                why = None
        elif t - self.t_plan >= self.min_gap:
            if not path_clear(self.grid, self.path[max(0, self.follower.seg):], self.r_block):
                why = "path blocked"
            elif self.follower.cross > 0.5:
                why = "off the path"
            elif self.follower.s > self.s_best + 0.05:
                self.s_best = self.follower.s; self.t_progress = t
            elif t - self.t_progress > self.replan_every:
                why = "no progress"
        if why is not None:
            self._plan(t, x, y, why)
        if self.follower is None:
            return 0.0, 0.0, self.status
        vx, wz = self.follower.update(x, y, yaw)
        if self.follower.done:                                                # reached the end of the planned path
            self.log.append((t, "waypoint %d reached at (%.2f, %.2f)" % (self.gi, x, y)))
            self.gi += 1; self.path = None; self.follower = None
            if self.goal() is None:
                self.status = "done"; return 0.0, 0.0, "done"
            return 0.0, 0.0, "run"                                            # the next tick plans the next leg
        return vx * scale, wz, "run"


# ======================================================================================================
def obstacle_models_sdf(circles, boxes, height=0.6):
    """SDF <model> blocks for static obstacles (cylinders and boxes), to be inserted before </world>."""
    out = []
    mat = "<material><ambient>0.85 0.45 0.1 1</ambient><diffuse>0.85 0.45 0.1 1</diffuse></material>"
    for i, (cx, cy, r) in enumerate(circles):
        out.append(
            '    <model name="obs_cyl_%d"><static>true</static><pose>%.4f %.4f %.4f 0 0 0</pose><link name="link">'
            '<collision name="c"><geometry><cylinder><radius>%.4f</radius><length>%.4f</length></cylinder></geometry></collision>'
            '<visual name="v"><geometry><cylinder><radius>%.4f</radius><length>%.4f</length></cylinder></geometry>%s</visual></link></model>'
            % (i, cx, cy, height / 2, r, height, r, height, mat))
    for i, (cx, cy, w, h) in enumerate(boxes):
        out.append(
            '    <model name="obs_box_%d"><static>true</static><pose>%.4f %.4f %.4f 0 0 0</pose><link name="link">'
            '<collision name="c"><geometry><box><size>%.4f %.4f %.4f</size></box></geometry></collision>'
            '<visual name="v"><geometry><box><size>%.4f %.4f %.4f</size></box></geometry>%s</visual></link></model>'
            % (i, cx, cy, height / 2, w, h, height, w, h, height, mat))
    return "\n".join(out)
