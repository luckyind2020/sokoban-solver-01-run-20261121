// Sokoban solver — make.fun sokoban-solver-01-run-20261121.
//
// Design (written from scratch for this bounty):
// The generator builds each level as a hallway plus 1-4 small rooms, each room
// holding 3-4 boxes and as many goals, connected to the hallway through a
// single door cell. A box pushed through a door into the hallway can never
// come back, so every box stays in its room and the level's push-optimal
// count is the sum of the rooms' optima. This solver therefore:
//   1. finds the rooms by articulation-point analysis on the floor graph,
//   2. solves each room independently with A* over push states,
//      - heuristic: minimum-cost assignment of boxes to goals (Hungarian
//        over precomputed shortest-path distances, admissible),
//      - deadlock pruning: single-box pull-reachability (a box on a cell
//        from which no goal is pull-reachable can never be solved) plus the
//        classic frozen 2x2 wall/box block rule,
//      - player positions canonicalised to their flood-fill region,
//   3. stitches the room solutions together with hallway walks.
// A deterministic beam search is the fallback when A* exhausts its
// expansion budget. Determinism: all budgets are counted in node
// expansions, never wall-clock time; no randomness anywhere, so two runs
// on the same set produce byte-identical output.
//
// Usage: node main.ts [inputRoot] [outputRoot]   (defaults /input /output)
// Reads <inputRoot>/<id>/level.txt, writes <outputRoot>/<id>/solution.txt
// with the player's moves as u/d/l/r letters.
'use strict';

const fs = require('node:fs');
const path = require('node:path');

const DX = [0, 0, -1, 1];
const DY = [-1, 1, 0, 0];
const CH = ['u', 'd', 'l', 'r'];

// ---------------------------------------------------------------- level ---

function parseLevel(text) {
  const rawLines = text.replace(/\r/g, '').split('\n');
  while (rawLines.length > 0 && rawLines[rawLines.length - 1] === '') rawLines.pop();
  const h = rawLines.length;
  let w = 0;
  for (const line of rawLines) if (line.length > w) w = line.length;
  const walls = new Uint8Array(w * h); // 1 = wall
  const goals = new Set();
  const boxes = [];
  const floor = [];
  let player = -1;
  for (let y = 0; y < h; y++) {
    const line = rawLines[y];
    for (let x = 0; x < w; x++) {
      const c = x < line.length ? line[x] : '#';
      const i = y * w + x;
      if (c === '#') { walls[i] = 1; continue; }
      floor.push(i);
      if (c === '.' || c === '*' || c === '+') goals.add(i);
      if (c === '$' || c === '*') boxes.push(i);
      if (c === '@' || c === '+') player = i;
    }
  }
  boxes.sort((a, b) => a - b);
  return { w, h, walls, goals, boxes, player, floor };
}

// ------------------------------------------------------------ grid utils ---

// Connected component of `start` over cells where pass(i) is true.
function component(start, pass, w, h) {
  const seen = new Set([start]);
  const stack = [start];
  while (stack.length > 0) {
    const c = stack.pop();
    const x = c % w;
    const y = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = x + DX[d];
      const ny = y + DY[d];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const n = ny * w + nx;
      if (!pass(n) || seen.has(n)) continue;
      seen.add(n);
      stack.push(n);
    }
  }
  return seen;
}

// Flood fill from `start` over passable cells avoiding `blocked`.
// Returns { set, min, parent } where parent[c] is the previous cell on the
// BFS tree from start (-1 for start itself).
function flood(ctx, start, blocked) {
  const w = ctx.w;
  const h = ctx.h;
  const parent = new Int32Array(w * h).fill(-1);
  const set = new Set([start]);
  parent[start] = -2;
  const queue = [start];
  let min = start;
  let qi = 0;
  while (qi < queue.length) {
    const c = queue[qi++];
    if (c < min) min = c;
    const x = c % w;
    const y = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = x + DX[d];
      const ny = y + DY[d];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const n = ny * w + nx;
      if (!ctx.pass[n] || set.has(n) || blocked.has(n)) continue;
      set.add(n);
      parent[n] = c;
      queue.push(n);
    }
  }
  return { set, min, parent };
}

// Shortest walk (as move letters) from `from` to `to` over passable cells
// avoiding `blocked`; null when unreachable. Uses flood()'s BFS tree.
function walkPath(ctx, from, to, blocked) {
  if (from === to) return '';
  const fl = flood(ctx, from, blocked);
  if (!fl.set.has(to)) return null;
  const letters = [];
  let c = to;
  while (c !== from) {
    const p = fl.parent[c];
    const dx = (c % ctx.w) - (p % ctx.w);
    const dy = ((c / ctx.w) | 0) - ((p / ctx.w) | 0);
    letters.push(dx === 1 ? 'r' : dx === -1 ? 'l' : dy === 1 ? 'd' : 'u');
    c = p;
  }
  letters.reverse();
  return letters.join('');
}

// BFS distances from `from` over passable cells (ignoring boxes).
function bfsDist(ctx, from) {
  const dist = new Int32Array(ctx.w * ctx.h).fill(-1);
  dist[from] = 0;
  const queue = [from];
  let qi = 0;
  while (qi < queue.length) {
    const c = queue[qi++];
    const x = c % ctx.w;
    const y = (c / ctx.w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = x + DX[d];
      const ny = y + DY[d];
      if (nx < 0 || ny < 0 || nx >= ctx.w || ny >= ctx.h) continue;
      const n = ny * ctx.w + nx;
      if (!ctx.pass[n] || dist[n] !== -1) continue;
      dist[n] = dist[c] + 1;
      queue.push(n);
    }
  }
  return dist;
}

// ------------------------------------------------------- room detection ---

// Find rooms: a room is a set of floor cells separated from the player's
// component by removing a single door cell, holding 3-4 boxes and as many
// goals. Returns [{ cells, door, boxes, goals, approach, arena }].
function findRooms(level) {
  const w = level.w;
  const h = level.h;
  const floorSet = new Set(level.floor);
  const player = level.player;
  const cands = [];
  for (const c of level.floor) {
    if (c === player) continue;
    // Components of floor \ {c}.
    const pass = (i) => i !== c && floorSet.has(i);
    const seen = new Set([c]);
    const comps = [];
    for (const f of level.floor) {
      if (seen.has(f)) continue;
      const comp = component(f, pass, w, h);
      for (const x of comp) seen.add(x);
      comps.push(comp);
    }
    if (comps.length < 2) continue;
    const playerComp = comps.find((C) => C.has(player));
    if (!playerComp) continue;
    for (const C of comps) {
      if (C === playerComp) continue;
      const boxes = level.boxes.filter((b) => C.has(b) || b === c);
      const goals = [...level.goals].filter((g) => C.has(g) || g === c);
      if (boxes.length >= 2 && boxes.length === goals.length && C.size <= 40) {
        cands.push({ cells: C, door: c, boxes, goals, playerComp });
      }
    }
  }
  // Resolve overlaps (an inner articulation can isolate a sub-part of a real
  // room): keep the largest candidates, all pairwise cell-disjoint.
  cands.sort((a, b) => b.cells.size - a.cells.size || a.door - b.door);
  const rooms = [];
  const used = new Set();
  for (const cand of cands) {
    let overlap = false;
    for (const x of cand.cells) if (used.has(x)) { overlap = true; break; }
    if (used.has(cand.door)) overlap = true;
    if (overlap) continue;
    rooms.push(cand);
    for (const x of cand.cells) used.add(x);
    used.add(cand.door);
  }
  // Approach cell: a hallway cell adjacent to the door, preferably aligned
  // so pushing from it sends a door box into the room.
  for (const room of rooms) {
    const door = room.door;
    const dx = door % w;
    const dy = (door / w) | 0;
    const adj = [];
    for (let d = 0; d < 4; d++) {
      const nx = dx + DX[d];
      const ny = dy + DY[d];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const n = ny * w + nx;
      if (room.playerComp.has(n)) adj.push(n);
    }
    adj.sort((a, b) => a - b);
    let approach = adj.length > 0 ? adj[0] : -1;
    for (const n of adj) {
      const beyond = door + (door - n);
      if (room.cells.has(beyond)) { approach = n; break; }
    }
    room.approach = approach;
    const arena = new Set(room.cells);
    arena.add(door);
    for (const n of adj) arena.add(n);
    room.arena = arena;
    delete room.playerComp;
  }
  return rooms;
}

// ------------------------------------------------------------ solver ctx ---

// Build the search context for one arena (room or whole level).
function makeCtx(level, arenaCells, goalsArr, door, needExit) {
  const pass = new Uint8Array(level.w * level.h);
  for (const c of arenaCells) pass[c] = 1;
  const ctx = {
    w: level.w,
    h: level.h,
    pass,
    goals: goalsArr.slice().sort((a, b) => a - b),
    goalSet: new Set(goalsArr),
    door,
    needExit,
    budget: null, // set per search
  };
  // Shortest-path distances from every goal (ignoring boxes).
  ctx.distToGoal = ctx.goals.map((g) => bfsDist(ctx, g));
  // Pull-reachability: cells from which a box can reach a goal by pulls,
  // ignoring other boxes and player reachability (a necessary condition, so
  // pruning a box outside this set is sound).
  ctx.pullReach = new Uint8Array(level.w * level.h);
  const queue = [];
  for (const g of ctx.goals) {
    ctx.pullReach[g] = 1;
    queue.push(g);
  }
  let qi = 0;
  while (qi < queue.length) {
    const p = queue[qi++];
    const px = p % ctx.w;
    const py = (p / ctx.w) | 0;
    // A box at q can reach p in one forward push exactly when q = p - d
    // (push direction d) and the player can stand at p - 2d.
    for (let d = 0; d < 4; d++) {
      const qx = px - DX[d];
      const qy = py - DY[d];
      const sx = px - 2 * DX[d];
      const sy = py - 2 * DY[d];
      if (qx < 0 || qy < 0 || qx >= ctx.w || qy >= ctx.h) continue;
      if (sx < 0 || sy < 0 || sx >= ctx.w || sy >= ctx.h) continue;
      const q = qy * ctx.w + qx; // box comes from here
      const s = sy * ctx.w + sx; // player stance for that push
      if (!ctx.pass[q] || !ctx.pass[s] || ctx.pullReach[q]) continue;
      ctx.pullReach[q] = 1;
      queue.push(q);
    }
  }
  return ctx;
}

// Admissible heuristic: minimum-cost assignment of boxes to goals over
// shortest-path distances (brute-force permutations for n <= 5, otherwise
// the sum of per-box minima, still admissible).
function heuristic(ctx, boxes) {
  const n = boxes.length;
  if (n === 0) return 0;
  if (n <= 5) {
    let best = Infinity;
    const idx = [];
    for (let i = 0; i < n; i++) idx.push(i);
    const permute = (k) => {
      if (k === n) {
        let s = 0;
        for (let i = 0; i < n; i++) {
          const d = ctx.distToGoal[idx[i]][boxes[i]];
          if (d < 0) { s = Infinity; break; }
          s += d;
        }
        if (s < best) best = s;
        return;
      }
      for (let i = k; i < n; i++) {
        const t = idx[k]; idx[k] = idx[i]; idx[i] = t;
        permute(k + 1);
        const t2 = idx[k]; idx[k] = idx[i]; idx[i] = t2;
      }
    };
    permute(0);
    return best;
  }
  let s = 0;
  for (const b of boxes) {
    let m = Infinity;
    for (let g = 0; g < n; g++) {
      const d = ctx.distToGoal[g][b];
      if (d >= 0 && d < m) m = d;
    }
    s += m;
  }
  return s;
}

// Sound deadlock test for a box configuration (boxes sorted, boxSet given).
function isDead(ctx, boxes, boxSet) {
  for (const b of boxes) {
    if (ctx.goalSet.has(b)) continue;
    if (!ctx.pullReach[b]) return true; // can never reach any goal
    const x = b % ctx.w;
    const y = (b / ctx.w) | 0;
    // Frozen 2x2 block of walls/boxes containing an off-goal box.
    for (let ox = -1; ox <= 0; ox++) {
      for (let oy = -1; oy <= 0; oy++) {
        const sx = x + ox;
        const sy = y + oy;
        if (sx < 0 || sy < 0 || sx + 1 >= ctx.w || sy + 1 >= ctx.h) continue;
        const cells = [
          sy * ctx.w + sx, sy * ctx.w + sx + 1,
          (sy + 1) * ctx.w + sx, (sy + 1) * ctx.w + sx + 1,
        ];
        let full = true;
        for (const cc of cells) {
          if (ctx.pass[cc] && !boxSet.has(cc)) { full = false; break; }
        }
        if (full) return true;
      }
    }
  }
  return false;
}

function stateKey(boxes, regionMin) {
  return boxes.join(',') + '|' + regionMin;
}

function isGoalState(ctx, boxes, player) {
  for (let i = 0; i < boxes.length; i++) {
    if (boxes[i] !== ctx.goals[i]) return false;
  }
  if (ctx.needExit && ctx.door >= 0) {
    const fl = flood(ctx, player, new Set(boxes));
    if (!fl.set.has(ctx.door)) return false;
  }
  return true;
}

// Generate all legal pushes from a state. Each successor carries the move
// segment (walk to the stance cell + the push letter).
function successors(ctx, boxes, player) {
  const boxSet = new Set(boxes);
  const fl = flood(ctx, player, boxSet);
  const out = [];
  for (let bi = 0; bi < boxes.length; bi++) {
    const b = boxes[bi];
    const bx = b % ctx.w;
    const by = (b / ctx.w) | 0;
    for (let d = 0; d < 4; d++) {
      const destX = bx + DX[d];
      const destY = by + DY[d];
      const stX = bx - DX[d];
      const stY = by - DY[d];
      if (destX < 0 || destY < 0 || destX >= ctx.w || destY >= ctx.h) continue;
      if (stX < 0 || stY < 0 || stX >= ctx.w || stY >= ctx.h) continue;
      const dest = destY * ctx.w + destX;
      const stance = stY * ctx.w + stX;
      if (!ctx.pass[dest] || boxSet.has(dest)) continue;
      if (!fl.set.has(stance)) continue;
      const nb = boxes.slice();
      nb[bi] = dest;
      nb.sort((a, b2) => a - b2);
      const nbSet = new Set(nb);
      if (isDead(ctx, nb, nbSet)) continue;
      const walk = walkPath(ctx, player, stance, boxSet);
      if (walk === null) continue;
      const region = flood(ctx, b, nbSet);
      out.push({
        boxes: nb,
        player: b,
        seg: walk + CH[d],
        key: stateKey(nb, region.min),
        h: heuristic(ctx, nb),
      });
    }
  }
  return out;
}

// ------------------------------------------------------------- binary heap ---

function heapPush(heap, item) {
  heap.push(item);
  let i = heap.length - 1;
  while (i > 0) {
    const p = ((i - 1) / 2) | 0;
    if (cmpHeap(heap[p], heap[i]) <= 0) break;
    const t = heap[p]; heap[p] = heap[i]; heap[i] = t;
    i = p;
  }
}

function cmpHeap(a, b) {
  if (a.f !== b.f) return a.f - b.f;
  if (a.h !== b.h) return a.h - b.h;
  return a.seq - b.seq;
}

function heapPop(heap) {
  const top = heap[0];
  const last = heap.pop();
  if (heap.length > 0) {
    heap[0] = last;
    let i = 0;
    for (;;) {
      const l = 2 * i + 1;
      const r = 2 * i + 2;
      let m = i;
      if (l < heap.length && cmpHeap(heap[l], heap[m]) < 0) m = l;
      if (r < heap.length && cmpHeap(heap[r], heap[m]) < 0) m = r;
      if (m === i) break;
      const t = heap[m]; heap[m] = heap[i]; heap[i] = t;
      i = m;
    }
  }
  return top;
}

// ------------------------------------------------------------------ A* ---

// Optimal push search. Returns { moves, pushes, finalPlayer } or null when
// the budget is exhausted or no solution exists. `budget` is a shared
// { used, limit } counter (deterministic, no clock).
function astar(ctx, boxes0, player0, budget) {
  const boxSet0 = new Set(boxes0);
  if (isDead(ctx, boxes0, boxSet0)) return null;
  const startRegion = flood(ctx, player0, boxSet0);
  const startKey = stateKey(boxes0, startRegion.min);
  const nodes = new Map();
  nodes.set(startKey, { boxes: boxes0, player: player0, g: 0, parent: null, seg: '' });
  const heap = [];
  const h0 = heuristic(ctx, boxes0);
  let seq = 0;
  heapPush(heap, { f: h0, h: h0, g: 0, seq: seq++, key: startKey });
  let goalNode = null;
  while (heap.length > 0) {
    const cur = heapPop(heap);
    const node = nodes.get(cur.key);
    if (!node || node.g !== cur.g) continue; // stale heap entry
    if (budget.used >= budget.limit) return null;
    budget.used++;
    if (isGoalState(ctx, node.boxes, node.player)) { goalNode = node; break; }
    for (const s of successors(ctx, node.boxes, node.player)) {
      const g = node.g + 1;
      const prev = nodes.get(s.key);
      if (prev && prev.g <= g) continue;
      nodes.set(s.key, { boxes: s.boxes, player: s.player, g, parent: cur.key, seg: s.seg });
      heapPush(heap, { f: g + s.h, h: s.h, g, seq: seq++, key: s.key });
    }
  }
  if (!goalNode) return null;
  const parts = [];
  let n = goalNode;
  while (n.parent !== null) {
    parts.push(n.seg);
    n = nodes.get(n.parent);
  }
  parts.reverse();
  return { moves: parts.join(''), pushes: goalNode.g, finalPlayer: goalNode.player };
}

// ------------------------------------------------------- beam (fallback) ---

// Deterministic beam search over push states. Returns a valid (not
// necessarily optimal) solution, or null. `nodeCap` bounds total states.
function beamSolve(ctx, boxes0, player0, width, maxDepth, nodeCap) {
  const boxSet0 = new Set(boxes0);
  if (isDead(ctx, boxes0, boxSet0)) return null;
  const store = new Map(); // id -> node
  let nextId = 1;
  const startRegion = flood(ctx, player0, boxSet0);
  const startKey = stateKey(boxes0, startRegion.min);
  store.set(0, { id: 0, boxes: boxes0, player: player0, g: 0, parent: -1, seg: '', key: startKey });
  let levelNodes = [0];
  let total = 1;
  for (let depth = 0; depth < maxDepth; depth++) {
    // Goal check on this level.
    for (const id of levelNodes) {
      const n = store.get(id);
      if (isGoalState(ctx, n.boxes, n.player)) {
        const parts = [];
        let m = n;
        while (m.parent >= 0) { parts.push(m.seg); m = store.get(m.parent); }
        parts.reverse();
        return { moves: parts.join(''), pushes: n.g, finalPlayer: n.player };
      }
    }
    const cands = [];
    for (const id of levelNodes) {
      const n = store.get(id);
      for (const s of successors(ctx, n.boxes, n.player)) {
        cands.push({ boxes: s.boxes, player: s.player, seg: s.seg, key: s.key, h: s.h, parent: id, g: n.g + 1 });
      }
      if (total > nodeCap) return null;
    }
    // Dedup by key (keep lowest h), sort deterministically, keep width.
    cands.sort((a, b) => (a.g + a.h) - (b.g + b.h) || a.h - b.h || (a.key < b.key ? -1 : a.key > b.key ? 1 : 0));
    const seen = new Set();
    const nextLevel = [];
    for (const c of cands) {
      if (seen.has(c.key)) continue;
      seen.add(c.key);
      const id = nextId++;
      store.set(id, { id, boxes: c.boxes, player: c.player, g: c.g, parent: c.parent, seg: c.seg, key: c.key });
      nextLevel.push(id);
      total++;
      if (nextLevel.length >= width) break;
      if (total > nodeCap) return null;
    }
    if (nextLevel.length === 0) return null;
    levelNodes = nextLevel;
  }
  return null;
}

// ---------------------------------------------------------- level solve ---

// Solve one room (arena = room cells + door + hallway cells by the door).
// Tries A* first, then the beam fallback. Returns a solution or null.
function solveRoom(level, room, needExit, budget) {
  const ctx = makeCtx(level, room.arena, room.goals, room.door, needExit);
  ctx.budget = budget;
  let sol = astar(ctx, room.boxes.slice().sort((a, b) => a - b), room.approach, budget);
  if (!sol) {
    sol = beamSolve(ctx, room.boxes.slice().sort((a, b) => a - b), room.approach, 1024, 200, 300000);
  }
  return sol;
}

// Whole-level A* fallback (no room decomposition).
function solveWhole(level, budget) {
  const ctx = makeCtx(level, new Set(level.floor), [...level.goals], -1, false);
  ctx.budget = budget;
  let sol = astar(ctx, level.boxes.slice(), level.player, budget);
  if (!sol) {
    sol = beamSolve(ctx, level.boxes.slice(), level.player, 1024, 200, 300000);
  }
  return sol;
}

// All orderings of rooms (<= 24 for 4 rooms), deterministic.
function permutations(arr) {
  if (arr.length <= 1) return [arr.slice()];
  const out = [];
  for (let i = 0; i < arr.length; i++) {
    const rest = arr.slice(0, i).concat(arr.slice(i + 1));
    for (const p of permutations(rest)) out.push([arr[i]].concat(p));
  }
  return out;
}

function solveLevel(level, budget) {
  const rooms = findRooms(level);
  const coveredBoxes = new Set();
  const coveredGoals = new Set();
  for (const r of rooms) {
    for (const b of r.boxes) coveredBoxes.add(b);
    for (const g of r.goals) coveredGoals.add(g);
  }
  const covered = rooms.length > 0 &&
    coveredBoxes.size === level.boxes.length &&
    coveredGoals.size === level.goals.size &&
    rooms.every((r) => r.approach >= 0);
  if (covered) {
    // Solve every room both ways (cached, order-independent): with the
    // exit condition (needed unless the room is solved last) and without.
    const solved = rooms.map((room) => {
      const exit = solveRoom(level, room, true, budget);
      const plain = exit ? solveRoom(level, room, false, budget) : solveRoom(level, room, false, budget);
      return { room, exit, plain };
    });
    if (solved.every((s) => s.exit || s.plain)) {
      const idxs = rooms.map((_, i) => i);
      for (const order of permutations(idxs)) {
        const moves = stitch(level, solved, order);
        if (moves !== null) return moves;
      }
    }
  }
  const whole = solveWhole(level, budget);
  return whole ? whole.moves : '';
}

// Stitch room solutions in the given order: walk between approach cells,
// then each room's moves. Non-last rooms must use their exit solution;
// the last room uses its plain (fewest-push) solution when available.
// Returns the full move string, or null when a walk is impossible.
function stitch(level, solved, order) {
  let moves = '';
  let curPlayer = level.player;
  const curBoxes = new Set(level.boxes);
  for (let k = 0; k < order.length; k++) {
    const s = solved[order[k]];
    const last = k === order.length - 1;
    const sol = last ? (s.plain || s.exit) : s.exit;
    if (!sol) return null;
    const walk = walkPath(
      { w: level.w, h: level.h, pass: passAll(level) },
      curPlayer, s.room.approach, curBoxes,
    );
    if (walk === null) return null;
    moves += walk + sol.moves;
    curPlayer = sol.finalPlayer;
    for (const b of s.room.boxes) curBoxes.delete(b);
    for (const g of s.room.goals) curBoxes.add(g);
  }
  return moves;
}

let passAllCache = null;
function passAll(level) {
  if (passAllCache && passAllCache.level === level) return passAllCache.pass;
  const pass = new Uint8Array(level.w * level.h);
  for (const c of level.floor) pass[c] = 1;
  passAllCache = { level, pass };
  return pass;
}

// ----------------------------------------------------------------- main ---

function main() {
  // The harness mounts the set at /input and /output; its process driver
  // stands those mounts in with MAKEFUN_INPUT_DIR / MAKEFUN_OUTPUT_DIR.
  // (Optional argv overrides exist only for local testing.)
  const inputRoot = process.argv[2] || process.env.MAKEFUN_INPUT_DIR || '/input';
  const outputRoot = process.argv[3] || process.env.MAKEFUN_OUTPUT_DIR || '/output';
  const ids = fs.readdirSync(inputRoot)
    .filter((name) => {
      try { return fs.statSync(path.join(inputRoot, name)).isDirectory(); } catch (_) { return false; }
    })
    .sort();
  for (const id of ids) {
    const levelPath = path.join(inputRoot, id, 'level.txt');
    let text;
    try {
      text = fs.readFileSync(levelPath, 'utf8');
    } catch (_) {
      continue;
    }
    const level = parseLevel(text);
    // Fresh expansion budget per instance (deterministic, no clock).
    const budget = { used: 0, limit: 800000 };
    const moves = solveLevel(level, budget);
    fs.mkdirSync(path.join(outputRoot, id), { recursive: true });
    fs.writeFileSync(path.join(outputRoot, id, 'solution.txt'), moves, 'utf8');
  }
}

main();
