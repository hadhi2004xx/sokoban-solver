// Sokoban solver — Argon UBH, sokoban-solver-01-run-20261121.
// Pass 1: I/O + room decomposition + push-BFS (push-optimal, basic pruning).
// Pass 2: A* with a min-cost box->goal matching heuristic (admissible, so the
// first goal found is still push-optimal), plain push-BFS as fallback.
// Deterministic: fixed work budgets only, never the wall clock, no randomness.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

const DIRS = [
  { dr: -1, dc: 0, ch: 'u' },
  { dr: 1, dc: 0, ch: 'd' },
  { dr: 0, dc: -1, ch: 'l' },
  { dr: 0, dc: 1, ch: 'r' },
];
// Fixed work budgets per room (deterministic, never clock-based).
const MAX_EXPANSIONS_ASTAR = 400000;
const MAX_EXPANSIONS_BFS = 2000000;

function parseLevel(text) {
  const lines = text.replace(/\r/g, '').split('\n');
  while (lines.length && lines[lines.length - 1] === '') lines.pop();
  const H = lines.length;
  const W = Math.max(...lines.map((l) => l.length));
  const wall = new Uint8Array(W * H);
  const goal = new Uint8Array(W * H);
  const boxes = [];
  let player = -1;
  for (let r = 0; r < H; r++) {
    for (let c = 0; c < W; c++) {
      const ch = c < lines[r].length ? lines[r][c] : '#';
      const i = r * W + c;
      if (ch === '#') wall[i] = 1;
      else {
        if (ch === '.' || ch === '*' || ch === '+') goal[i] = 1;
        if (ch === '$' || ch === '*') boxes.push(i);
        if (ch === '@' || ch === '+') player = i;
      }
    }
  }
  boxes.sort((a, b) => a - b);
  return { W, H, wall, goal, boxes, player };
}

// Split the level into rooms. The hallway is the player's row (fallback: column).
// Each room is one connected component of non-hallway floor, entered through a
// single door cell. The player enters a room from the hallway cell next to the
// door (the "entry" cell); a box may sit on the door cell and be pushed in.
function decompose(L) {
  const { W, H, wall, player } = L;
  const pr = Math.floor(player / W);
  const pc = player % W;
  const byRow = decomposeAxis(L, 'row', pr);
  if (byRow) return byRow;
  return decomposeAxis(L, 'col', pc);
}

function decomposeAxis(L, axis, line) {
  const { W, H, wall } = L;
  const isHall = (r, c) => (axis === 'row' ? r === line : c === line);
  const doors = [];
  for (let r = 0; r < H; r++) {
    for (let c = 0; c < W; c++) {
      if (wall[r * W + c] || !isHall(r, c)) continue;
      const nbs = axis === 'row' ? [[r - 1, c], [r + 1, c]] : [[r, c - 1], [r, c + 1]];
      for (const [nr, nc] of nbs) {
        if (nr < 0 || nr >= H || nc < 0 || nc >= W) continue;
        if (!wall[nr * W + nc]) doors.push(nr * W + nc);
      }
    }
  }
  if (!doors.length) return null;
  const seen = new Uint8Array(W * H);
  const rooms = [];
  for (const d of doors) {
    if (seen[d]) continue;
    const cells = [];
    const q = [d];
    seen[d] = 1;
    while (q.length) {
      const x = q.pop();
      cells.push(x);
      const r = Math.floor(x / W);
      const c = x % W;
      for (const { dr, dc } of DIRS) {
        const nr = r + dr;
        const nc = c + dc;
        if (nr < 0 || nr >= H || nc < 0 || nc >= W) continue;
        if (isHall(nr, nc)) continue;
        const y = nr * W + nc;
        if (wall[y] || seen[y]) continue;
        seen[y] = 1;
        q.push(y);
      }
    }
    // Entry cell: the hallway cell adjacent to the door.
    const dr0 = Math.floor(d / W);
    const dc0 = d % W;
    const entryCell = axis === 'row' ? line * W + dc0 : dr0 * W + line;
    rooms.push({ door: d, cells, entry: entryCell, axis, line });
  }
  return { rooms, axis, line };
}

// Flood fill over `space` avoiding boxes; returns the reachable set and the
// smallest cell index in it (canonical player position).
function region(L, space, boxesSet, start) {
  const { W, H } = L;
  const seen = new Set([start]);
  const q = [start];
  let min = start;
  while (q.length) {
    const x = q.pop();
    if (x < min) min = x;
    const r = Math.floor(x / W);
    const c = x % W;
    for (const { dr, dc } of DIRS) {
      const nr = r + dr;
      const nc = c + dc;
      if (nr < 0 || nr >= H || nc < 0 || nc >= W) continue;
      const y = nr * W + nc;
      if (!space.has(y) || boxesSet.has(y) || seen.has(y)) continue;
      seen.add(y);
      q.push(y);
    }
  }
  return { min, set: seen };
}

// BFS path (move letters) from `from` to `to` over `space` avoiding boxes.
function bfsPath(L, space, boxesSet, from, to) {
  if (from === to) return '';
  const { W, H } = L;
  const prev = new Map([[from, null]]);
  const q = [from];
  let head = 0;
  while (head < q.length) {
    const x = q[head++];
    const r = Math.floor(x / W);
    const c = x % W;
    for (let di = 0; di < 4; di++) {
      const { dr, dc, ch } = DIRS[di];
      const nr = r + dr;
      const nc = c + dc;
      if (nr < 0 || nr >= H || nc < 0 || nc >= W) continue;
      const y = nr * W + nc;
      if (!space.has(y) || boxesSet.has(y) || prev.has(y)) continue;
      prev.set(y, [x, ch]);
      if (y === to) {
        let s = '';
        let cur = y;
        while (prev.get(cur) !== null) {
          const [p, m] = prev.get(cur);
          s = m + s;
          cur = p;
        }
        return s;
      }
      q.push(y);
    }
  }
  return null;
}

// All permutations of [0..n-1] as index arrays (n is 3 or 4 here).
function permutations(n) {
  const out = [];
  const a = [...Array(n).keys()];
  const gen = (k) => {
    if (k === 1) { out.push(a.slice()); return; }
    for (let i = 0; i < k; i++) {
      gen(k - 1);
      const j = k % 2 ? 0 : i;
      [a[j], a[k - 1]] = [a[k - 1], a[j]];
    }
  };
  gen(n);
  return out;
}

// Solve one room push-optimally from its entry cell. Returns the move string
// from the entry cell back to the entry cell with every box on a goal, or null.
function solveRoom(L, room) {
  const { W, H, wall, goal } = L;
  const roomSet = new Set(room.cells);
  const space = new Set(room.cells);
  space.add(room.entry);
  const boxes0 = L.boxes.filter((b) => roomSet.has(b)).sort((a, b) => a - b);
  const goals = room.cells.filter((x) => goal[x]);
  if (boxes0.length !== goals.length) return null;

  // Push distances between room cells (boxes ignored): BFS from each cell.
  const dist = new Map();
  for (const s of room.cells) {
    const d = new Map([[s, 0]]);
    const q = [s];
    let head = 0;
    while (head < q.length) {
      const x = q[head++];
      const r = Math.floor(x / W);
      const c = x % W;
      for (const { dr, dc } of DIRS) {
        const nr = r + dr;
        const nc = c + dc;
        if (nr < 0 || nr >= H || nc < 0 || nc >= W) continue;
        const y = nr * W + nc;
        if (!roomSet.has(y) || d.has(y)) continue;
        d.set(y, d.get(x) + 1);
        q.push(y);
      }
    }
    dist.set(s, d);
  }

  // Dead cells: reverse-push reachability from the goals (superset of live
  // cells; safe to prune). A box at x could have come from x-d pushed in
  // direction d, which needs the player to have stood at x-2d.
  const live = new Set(goals);
  const lq = [...goals];
  while (lq.length) {
    const x = lq.pop();
    const r = Math.floor(x / W);
    const c = x % W;
    for (const { dr, dc } of DIRS) {
      const yr = r - dr;
      const yc = c - dc;
      const pr2 = r - 2 * dr;
      const pc2 = c - 2 * dc;
      if (yr < 0 || yr >= H || yc < 0 || yc >= W) continue;
      if (pr2 < 0 || pr2 >= H || pc2 < 0 || pc2 >= W) continue;
      const y = yr * W + yc;
      if (!space.has(y) || live.has(y)) continue;
      if (!space.has(pr2 * W + pc2)) continue;
      live.add(y);
      lq.push(y);
    }
  }
  const dead = new Set(room.cells.filter((x) => !live.has(x)));

  // 2x2 deadlock: a 2x2 block of walls/boxes holding a box off-goal is frozen.
  const deadlock2x2 = (boxesSet) => {
    for (let r = 0; r < H - 1; r++) {
      for (let c = 0; c < W - 1; c++) {
        const cs = [r * W + c, (r + 1) * W + c, r * W + c + 1, (r + 1) * W + c + 1];
        let bad = false;
        let all = true;
        for (const x of cs) {
          if (!(wall[x] || boxesSet.has(x))) { all = false; break; }
          if (boxesSet.has(x) && !goal[x]) bad = true;
        }
        if (all && bad) return true;
      }
    }
    return false;
  };

  const isGoalState = (node) => {
    for (const b of node.boxes) if (!goal[b]) return false;
    const reg = region(L, space, new Set(node.boxes), node.player);
    return reg.set.has(room.entry);
  };

  const keyOf = (boxes, canon) => boxes.join(',') + '|' + canon;

  // Min-cost matching of boxes to goals over push distances (admissible:
  // one push moves one box one cell, so the matching cost drops by <= 1).
  const perms = permutations(boxes0.length);
  const heuristic = (boxes) => {
    let best = Infinity;
    for (const p of perms) {
      let s = 0;
      for (let i = 0; i < boxes.length; i++) {
        const d = dist.get(boxes[i]).get(goals[p[i]]);
        if (d === undefined) { s = Infinity; break; }
        s += d;
      }
      if (s < best) best = s;
    }
    return best;
  };

  // Successor pushes from a node: {boxes, player, canon, pushBox, pushDir}.
  function* successors(node) {
    const boxesSet = new Set(node.boxes);
    const reg = region(L, space, boxesSet, node.player);
    for (let bi = 0; bi < node.boxes.length; bi++) {
      const b = node.boxes[bi];
      const br = Math.floor(b / W);
      const bc = b % W;
      for (let di = 0; di < 4; di++) {
        const { dr, dc } = DIRS[di];
        const dest = (br + dr) * W + (bc + dc);
        const behind = (br - dr) * W + (bc - dc);
        if (!roomSet.has(dest) || boxesSet.has(dest)) continue;
        if (!space.has(behind) || !reg.set.has(behind)) continue;
        if (dead.has(dest)) continue;
        const nb = node.boxes.slice();
        nb[bi] = dest;
        nb.sort((a, b2) => a - b2);
        const nbSet = new Set(nb);
        if (deadlock2x2(nbSet)) continue;
        const nreg = region(L, space, nbSet, b);
        yield { boxes: nb, player: b, canon: nreg.min, pushBox: b, pushDir: di };
      }
    }
  }

  const startReg = region(L, space, new Set(boxes0), room.entry);
  const start = { boxes: boxes0, player: room.entry, canon: startReg.min, parent: null, pushBox: -1, pushDir: -1, g: 0 };

  let found = searchAStar();
  if (!found) found = searchBFS();
  if (!found) return null;

  function searchAStar() {
    // Binary heap ordered by (f, -g, seq): deterministic tie-breaking.
    const heap = [];
    let seq = 0;
    const cmp = (a, b) => (a.f - b.f) || (b.g - a.g) || (a.seq - b.seq);
    const heapPush = (n) => {
      heap.push(n);
      let i = heap.length - 1;
      while (i > 0) {
        const p = (i - 1) >> 1;
        if (cmp(heap[i], heap[p]) >= 0) break;
        [heap[i], heap[p]] = [heap[p], heap[i]];
        i = p;
      }
    };
    const heapPop = () => {
      const top = heap[0];
      const last = heap.pop();
      if (heap.length) {
        heap[0] = last;
        let i = 0;
        for (;;) {
          const l = 2 * i + 1;
          const r = l + 1;
          let m = i;
          if (l < heap.length && cmp(heap[l], heap[m]) < 0) m = l;
          if (r < heap.length && cmp(heap[r], heap[m]) < 0) m = r;
          if (m === i) break;
          [heap[i], heap[m]] = [heap[m], heap[i]];
          i = m;
        }
      }
      return top;
    };
    const best = new Map([[keyOf(start.boxes, start.canon), 0]]);
    const root = { ...start, h: heuristic(start.boxes), f: 0, seq: seq++ };
    root.f = root.h;
    heapPush(root);
    let expansions = 0;
    while (heap.length) {
      const node = heapPop();
      if (node.g > (best.get(keyOf(node.boxes, node.canon)) ?? Infinity)) continue;
      if (isGoalState(node)) return node;
      if (++expansions > MAX_EXPANSIONS_ASTAR) return null;
      for (const s of successors(node)) {
        const key = keyOf(s.boxes, s.canon);
        const g = node.g + 1;
        if (g >= (best.get(key) ?? Infinity)) continue;
        best.set(key, g);
        const h = heuristic(s.boxes);
        if (h === Infinity) continue;
        heapPush({ ...s, parent: node, g, h, f: g + h, seq: seq++ });
      }
    }
    return null;
  }

  function searchBFS() {
    const closed = new Set([keyOf(start.boxes, start.canon)]);
    const queue = [{ ...start }];
    let head = 0;
    let expansions = 0;
    while (head < queue.length) {
      const node = queue[head++];
      if (isGoalState(node)) return node;
      if (++expansions > MAX_EXPANSIONS_BFS) return null;
      for (const s of successors(node)) {
        const key = keyOf(s.boxes, s.canon);
        if (closed.has(key)) continue;
        closed.add(key);
        queue.push({ ...s, parent: node, g: node.g + 1 });
      }
    }
    return null;
  }

  // Reconstruct the push list, then replay it with walking moves.
  const pushes = [];
  for (let n = found; n.parent; n = n.parent) pushes.push({ box: n.pushBox, dir: n.pushDir });
  pushes.reverse();
  let player = room.entry;
  let boxes = boxes0.slice();
  let moves = '';
  for (const { box, dir } of pushes) {
    const { dr, dc, ch } = DIRS[dir];
    const br = Math.floor(box / W);
    const bc = box % W;
    const behind = (br - dr) * W + (bc - dc);
    const path = bfsPath(L, space, new Set(boxes), player, behind);
    if (path === null) return null;
    moves += path + ch;
    boxes = boxes.map((b) => (b === box ? (br + dr) * W + (bc + dc) : b));
    player = box;
  }
  const exit = bfsPath(L, space, new Set(boxes), player, room.entry);
  if (exit === null) return null;
  moves += exit;
  return moves;
}

function solveLevel(text) {
  const L = parseLevel(text);
  const dec = decompose(L);
  if (!dec) return '';
  // Solve rooms left-to-right by door column.
  const rooms = dec.rooms.slice().sort((a, b) => a.door - b.door);
  let player = L.player;
  let boxes = L.boxes.slice();
  let moves = '';
  const allSpace = new Set();
  for (let i = 0; i < L.W * L.H; i++) if (!L.wall[i]) allSpace.add(i);
  for (const room of rooms) {
    const path = bfsPath(L, allSpace, new Set(boxes), player, room.entry);
    if (path === null) return '';
    moves += path;
    const rm = solveRoom(L, room);
    if (rm === null) return '';
    moves += rm;
    player = room.entry;
    // Update global boxes: room boxes are now on goals.
    const roomGoals = room.cells.filter((x) => L.goal[x]);
    const roomBoxes = new Set(room.cells.filter((x) => boxes.includes(x)));
    boxes = boxes.filter((b) => !roomBoxes.has(b)).concat(roomGoals);
    boxes.sort((a, b) => a - b);
  }
  return moves;
}

const solve = (level) => solveLevel(level);

for (const id of readdirSync(I).sort()) {
  let moves = '';
  try {
    moves = solve(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  } catch {
    moves = '';
  }
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
