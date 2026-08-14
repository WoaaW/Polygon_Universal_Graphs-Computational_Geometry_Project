(function (global) {
  "use strict";

  var G = global.Geo;

  /**
   * Épaissit une ligne brisée en couloir.
   * @param {{x,y}[]} center ligne médiane
   * @param {number[]} halfWidth demi-largeur en chaque point
   * @returns {{left: {x,y}[], right: {x,y}[]}}
   */
  function thicken(center, halfWidth) {
    var left = [];
    var right = [];
    for (var i = 0; i < center.length; i++) {
      var prev = center[Math.max(0, i - 1)];
      var next = center[Math.min(center.length - 1, i + 1)];
      var dir = G.normalize(G.sub(next, prev));
      var n = { x: -dir.y, y: dir.x };
      var w = halfWidth[i];
      left.push({ x: center[i].x + n.x * w, y: center[i].y + n.y * w });
      right.push({ x: center[i].x - n.x * w, y: center[i].y - n.y * w });
    }
    return { left: left, right: right };
  }

  // Spirale Pair

  /**
   * @param {number} turns nombre de virages du couloir
   * @param {object} [o] { dTheta, decay, width, radius, center }
   * @returns polygone (Geo.Polygon), coupes, et indices des deux extrémités
   */
  function pairSpiral(turns, o) {
    o = o || {};
    var dTheta = o.dTheta || Math.PI / 2;
    var decay = o.decay || 0.62;
    var width = o.width === undefined ? 0.1 : o.width;
    var R = o.radius || 1;
    var c = o.center || { x: 0, y: 0 };

    var outer = [];
    var inner = [];
    for (var k = 0; k <= turns; k++) {
      var th = k * dTheta;
      var r = R * Math.pow(decay, k);
      var w = r * width;
      outer.push({
        x: c.x + (r + w) * Math.cos(th),
        y: c.y + (r + w) * Math.sin(th),
      });
      inner.push({
        x: c.x + (r - w) * Math.cos(th),
        y: c.y + (r - w) * Math.sin(th),
      });
    }
    var pts = outer.concat(inner.slice().reverse());

    var cuts = [];
    for (var j = 0; j < turns; j++) {
      cuts.push([G.mid(outer[j], outer[j + 1]), G.mid(inner[j], inner[j + 1])]);
    }

    return {
      polygon: new G.Polygon(pts),
      cuts: cuts,
      ends: [0, turns],
      outerCount: turns + 1,
      turns: turns,
    };
  }

  // Spirale Triple

  /**
   * Trois bras serpentins partant d'un moyeu central. La ligne médiane de
   * chaque bras s'éloigne du centre en oscillant : chaque oscillation coupe la
   * ligne de vue, donc coûte un lien.
   *
   * @param {number} bends nombre d'oscillations par bras
   */
  function tripleSpiral(bends, o) {
    o = o || {};
    var reach = o.reach || 1; // portée d'un bras
    var hub = (o.hub === undefined ? 0.22 : o.hub) * reach; // rayon du moyeu
    var swing = o.swing === undefined ? 0.25 : o.swing; // amplitude angulaire
    var width = o.width === undefined ? 0.035 : o.width;
    var c = o.center || { x: 0, y: 0 };

    var arms = [];
    for (var a = 0; a < 3; a++) {
      var base = (2 * Math.PI * a) / 3;
      var center = [];
      var hw = [];
      for (var k = 0; k <= bends; k++) {
        var s = k / bends;
        var r = hub + (reach - hub) * s;
        var th = base + swing * (k % 2 === 0 ? -1 : 1) * (0.35 + 0.65 * s);
        center.push({ x: c.x + r * Math.cos(th), y: c.y + r * Math.sin(th) });
        hw.push(width * reach);
      }
      var walls = thicken(center, hw);
      arms.push({ center: center, left: walls.left, right: walls.right });
    }

    var pts = [];
    for (var b = 0; b < 3; b++) {
      var arm = arms[b];
      for (var i = 0; i < arm.right.length; i++) pts.push(arm.right[i]);
      for (var j = arm.left.length - 1; j >= 0; j--) pts.push(arm.left[j]);
    }

    var cuts = [];
    var cutArm = [];
    for (var m = 0; m < 3; m++) {
      for (var n = 0; n < bends; n++) {
        cuts.push([
          G.mid(arms[m].left[n], arms[m].left[n + 1]),
          G.mid(arms[m].right[n], arms[m].right[n + 1]),
        ]);
        cutArm.push(m);
      }
    }

    var per = 2 * (bends + 1);
    var tips = [bends, per + bends, 2 * per + bends];

    return {
      polygon: new G.Polygon(pts),
      cuts: cuts,
      cutArm: cutArm,
      arms: arms,
      tips: tips,
      bends: bends,
    };
  }

  /** Le segment [a, b] est-il contenu dans P ? */
  function segmentInside(P, a, b) {
    var crosses = false;
    P.eachEdge(function (u, v) {
      if (!crosses && G.segmentsIntersect(a, b, u, v, true)) crosses = true;
    });
    if (crosses) return false;
    for (var t = 1; t < 8; t++) {
      if (P.locate(G.lerp(a, b, t / 8)) === "outside") return false;
    }
    return true;
  }

  /** Existe-t-il un segment intérieur touchant les deux coupes ? */
  function cutsSeeEachOther(P, A, B, samples) {
    samples = samples || 7;
    for (var i = 1; i < samples; i++) {
      var a = G.lerp(A[0], A[1], i / samples);
      for (var j = 1; j < samples; j++) {
        if (segmentInside(P, a, G.lerp(B[0], B[1], j / samples))) return true;
      }
    }
    return false;
  }

  /**
   * Sélectionne un sous-ensemble de coupes deux à deux invisibles.
   * @returns {{keep: number[], pairsTested: number}}
   */
  function certify(P, cuts, samples) {
    var keep = [];
    var tested = 0;
    for (var i = 0; i < cuts.length; i++) {
      var ok = true;
      for (var j = 0; j < keep.length && ok; j++) {
        tested++;
        if (cutsSeeEachOther(P, cuts[i], cuts[keep[j]], samples)) ok = false;
      }
      if (ok) keep.push(i);
    }
    return { keep: keep, pairsTested: tested };
  }

  function certifyArm(P, cuts, cutArm, arm, samples) {
    var sub = [];
    var map = [];
    cuts.forEach(function (c, i) {
      if (cutArm[i] === arm) {
        sub.push(c);
        map.push(i);
      }
    });
    var r = certify(P, sub, samples);
    return r.keep.map(function (i) {
      return map[i];
    });
  }

  // Vérification d'un chemin dessiné à la main

  /**
   * Un chemin (suite de points) est-il un dessin licite dans P ?
   * @returns {{ok: boolean, bad: number[]}} indices des segments fautifs
   */
  function checkPath(P, pts) {
    var bad = [];
    for (var i = 0; i + 1 < pts.length; i++) {
      if (!segmentInside(P, pts[i], pts[i + 1])) bad.push(i);
    }
    return { ok: bad.length === 0, bad: bad };
  }

  /** Nombre de coupes certifiées traversées par un chemin. */
  function cutsCrossed(pts, cuts, keep) {
    var hit = {};
    for (var i = 0; i + 1 < pts.length; i++) {
      keep.forEach(function (k) {
        var c = cuts[k];
        if (G.segmentsIntersect(pts[i], pts[i + 1], c[0], c[1], true))
          hit[k] = true;
      });
    }
    return Object.keys(hit).length;
  }

  function chamberDistance(P, cuts, keep, target, n) {
    n = n || 140;
    var b = P.bbox();
    var pad = Math.max(b.width, b.height) * 0.02;
    var x0 = b.minX - pad;
    var y0 = b.minY - pad;
    var dx = (b.width + 2 * pad) / (n - 1);
    var dy = (b.height + 2 * pad) / (n - 1);

    var inside = new Uint8Array(n * n);
    var i, j;
    for (j = 0; j < n; j++) {
      for (i = 0; i < n; i++) {
        var p = { x: x0 + i * dx, y: y0 + j * dy };
        inside[j * n + i] = P.locate(p) === "outside" ? 0 : 1;
      }
    }

    var active = keep.map(function (k) {
      var c = cuts[k];
      var m = G.mid(c[0], c[1]);
      return [
        G.add(m, G.mul(G.sub(c[0], m), 1.04)),
        G.add(m, G.mul(G.sub(c[1], m), 1.04)),
      ];
    });
    var dist = new Int32Array(n * n).fill(-1);

    // Cellule de départ : la plus proche de `target` parmi celles dans P.
    var si = Math.round((target.x - x0) / dx);
    var sj = Math.round((target.y - y0) / dy);
    var seed = -1;
    for (var rad = 0; rad < 10 && seed < 0; rad++) {
      for (var a = -rad; a <= rad && seed < 0; a++) {
        for (var c2 = -rad; c2 <= rad && seed < 0; c2++) {
          var ii = si + a;
          var jj = sj + c2;
          if (ii < 0 || jj < 0 || ii >= n || jj >= n) continue;
          if (inside[jj * n + ii]) seed = jj * n + ii;
        }
      }
    }
    var out = {
      dist: dist,
      w: n,
      h: n,
      x0: x0,
      y0: y0,
      dx: dx,
      dy: dy,
      inside: inside,
    };
    if (seed < 0) return out;

    var deque = new Int32Array(n * n * 4);
    var head = n * n * 2;
    var tail = head;
    dist[seed] = 0;
    deque[tail++] = seed;

    function stepCost(cur, nxt) {
      var pc = { x: x0 + (cur % n) * dx, y: y0 + ((cur - (cur % n)) / n) * dy };
      var pn = { x: x0 + (nxt % n) * dx, y: y0 + ((nxt - (nxt % n)) / n) * dy };
      var w = 0;
      for (var q = 0; q < active.length; q++) {
        if (G.segmentsIntersect(pc, pn, active[q][0], active[q][1], true)) w++;
      }
      return w;
    }

    while (head < tail) {
      var cur = deque[head++];
      var ci = cur % n;
      var cj = (cur - ci) / n;
      var nbrs = [
        ci > 0 ? cur - 1 : -1,
        ci < n - 1 ? cur + 1 : -1,
        cj > 0 ? cur - n : -1,
        cj < n - 1 ? cur + n : -1,
      ];
      for (var t = 0; t < 4; t++) {
        var nx = nbrs[t];
        if (nx < 0 || !inside[nx]) continue;
        var w = stepCost(cur, nx);
        var nd = dist[cur] + w;
        if (dist[nx] === -1 || nd < dist[nx]) {
          dist[nx] = nd;
          if (w === 0) deque[--head] = nx;
          else deque[tail++] = nx;
        }
      }
    }
    return out;
  }

  function sampleField(f, p) {
    var i0 = Math.round((p.x - f.x0) / f.dx);
    var j0 = Math.round((p.y - f.y0) / f.dy);
    for (var rad = 0; rad <= 3; rad++) {
      for (var a = -rad; a <= rad; a++) {
        for (var b = -rad; b <= rad; b++) {
          if (Math.max(Math.abs(a), Math.abs(b)) !== rad) continue;
          var i = i0 + a;
          var j = j0 + b;
          if (i < 0 || j < 0 || i >= f.w || j >= f.h) continue;
          var d = f.dist[j * f.w + i];
          if (d >= 0) return d;
        }
      }
    }
    return -1;
  }

  global.SP = {
    thicken: thicken,
    pairSpiral: pairSpiral,
    tripleSpiral: tripleSpiral,
    segmentInside: segmentInside,
    cutsSeeEachOther: cutsSeeEachOther,
    certify: certify,
    certifyArm: certifyArm,
    checkPath: checkPath,
    cutsCrossed: cutsCrossed,
    chamberDistance: chamberDistance,
    sampleField: sampleField,
  };
})(typeof window !== "undefined" ? window : globalThis);
