(function (global) {
  "use strict";

  var EPS = 1e-9;

  function pt(x, y) {
    if (Array.isArray(x)) return { x: x[0], y: x[1] };
    if (x !== null && typeof x === "object") return { x: x.x, y: x.y };
    return { x: x, y: y };
  }

  function add(a, b) {
    return { x: a.x + b.x, y: a.y + b.y };
  }
  function sub(a, b) {
    return { x: a.x - b.x, y: a.y - b.y };
  }
  function mul(a, k) {
    return { x: a.x * k, y: a.y * k };
  }
  function dot(a, b) {
    return a.x * b.x + a.y * b.y;
  }
  function cross(a, b) {
    return a.x * b.y - a.y * b.x;
  }
  function len(a) {
    return Math.hypot(a.x, a.y);
  }
  function dist(a, b) {
    return Math.hypot(a.x - b.x, a.y - b.y);
  }
  function dist2(a, b) {
    var dx = a.x - b.x,
      dy = a.y - b.y;
    return dx * dx + dy * dy;
  }
  function lerp2(a, b, t) {
    return { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
  }
  function mid(a, b) {
    return lerp2(a, b, 0.5);
  }

  function normalize(a) {
    var l = len(a);
    return l < EPS ? { x: 0, y: 0 } : mul(a, 1 / l);
  }

  function rotateAround(p, angle, c) {
    c = c || { x: 0, y: 0 };
    var co = Math.cos(angle),
      si = Math.sin(angle),
      d = sub(p, c);
    return { x: c.x + d.x * co - d.y * si, y: c.y + d.x * si + d.y * co };
  }

  function cross3(a, b, c) {
    return cross(sub(b, a), sub(c, a));
  }

  function orient(a, b, c, eps) {
    var v = cross3(a, b, c);
    eps = eps === undefined ? EPS : eps;
    return v > eps ? 1 : v < -eps ? -1 : 0;
  }

  // Vrai si p est sur le segment fermé [a, b].
  function onSegment(p, a, b, eps) {
    eps = eps === undefined ? EPS : eps;
    if (orient(a, b, p, eps) !== 0) return false;
    return (
      Math.min(a.x, b.x) - eps <= p.x &&
      p.x <= Math.max(a.x, b.x) + eps &&
      Math.min(a.y, b.y) - eps <= p.y &&
      p.y <= Math.max(a.y, b.y) + eps
    );
  }

  // Distance d'un point au segment [a, b].
  function distToSegment(p, a, b) {
    var ab = sub(b, a),
      l2 = dot(ab, ab);
    if (l2 < EPS) return dist(p, a);
    var t = Math.max(0, Math.min(1, dot(sub(p, a), ab) / l2));
    return dist(p, add(a, mul(ab, t)));
  }

  // Les segments [a,b] et [c,d] se coupent-ils ? (contacts inclus)
  function segmentsIntersect(a, b, c, d, proper) {
    var o1 = orient(a, b, c),
      o2 = orient(a, b, d);
    var o3 = orient(c, d, a),
      o4 = orient(c, d, b);
    if (proper) return o1 * o2 < 0 && o3 * o4 < 0;
    if (o1 !== o2 && o3 !== o4) return true;
    if (o1 === 0 && onSegment(c, a, b)) return true;
    if (o2 === 0 && onSegment(d, a, b)) return true;
    if (o3 === 0 && onSegment(a, c, d)) return true;
    if (o4 === 0 && onSegment(b, c, d)) return true;
    return false;
  }

  // Intersection des droites (a,b) et (c,d), ou null si parallèles.
  function lineIntersection(a, b, c, d) {
    var r = sub(b, a),
      s = sub(d, c),
      den = cross(r, s);
    if (Math.abs(den) < EPS) return null;
    return add(a, mul(r, cross(sub(c, a), s) / den));
  }

  // Intersection des segments [a,b] et [c,d], ou null.
  function segmentIntersection(a, b, c, d) {
    var p = lineIntersection(a, b, c, d);
    if (!p) return null;
    return onSegment(p, a, b) && onSegment(p, c, d) ? p : null;
  }

  function bbox(points) {
    if (!points.length) return null;
    var minX = Infinity,
      minY = Infinity,
      maxX = -Infinity,
      maxY = -Infinity;
    for (var i = 0; i < points.length; i++) {
      var p = points[i];
      if (p.x < minX) minX = p.x;
      if (p.y < minY) minY = p.y;
      if (p.x > maxX) maxX = p.x;
      if (p.y > maxY) maxY = p.y;
    }
    return {
      minX: minX,
      minY: minY,
      maxX: maxX,
      maxY: maxY,
      width: maxX - minX,
      height: maxY - minY,
    };
  }

  function convexHull(points) {
    var ps = points.slice().sort(function (p, q) {
      return p.x - q.x || p.y - q.y;
    });
    if (ps.length < 3) return ps;
    function half(src) {
      var out = [];
      for (var i = 0; i < src.length; i++) {
        while (
          out.length >= 2 &&
          orient(out[out.length - 2], out[out.length - 1], src[i]) <= 0
        )
          out.pop();
        out.push(src[i]);
      }
      out.pop();
      return out;
    }
    return half(ps).concat(half(ps.slice().reverse()));
  }

  function Polygon(points) {
    this.points = (points || []).map(function (p) {
      return pt(p);
    });
  }

  Polygon.from = function (coords) {
    return new Polygon(coords);
  };

  Polygon.regular = function (n, radius, center, startAngle) {
    center = center || { x: 0, y: 0 };
    startAngle = startAngle || 0;
    var ps = [];
    for (var i = 0; i < n; i++) {
      var a = startAngle + (2 * Math.PI * i) / n;
      ps.push({
        x: center.x + radius * Math.cos(a),
        y: center.y + radius * Math.sin(a),
      });
    }
    return new Polygon(ps);
  };

  Polygon.rectangle = function (x, y, w, h) {
    return new Polygon([
      [x, y],
      [x + w, y],
      [x + w, y + h],
      [x, y + h],
    ]);
  };

  Object.defineProperty(Polygon.prototype, "n", {
    get: function () {
      return this.points.length;
    },
  });

  // Sommet i, indexation cyclique (les indices négatifs marchent).
  Polygon.prototype.at = function (i) {
    var n = this.points.length;
    return this.points[((i % n) + n) % n];
  };

  // Arête i sous forme [origine, destination].
  Polygon.prototype.edge = function (i) {
    return [this.at(i), this.at(i + 1)];
  };

  // Parcourt les arêtes : callback(a, b, i).
  Polygon.prototype.eachEdge = function (fn) {
    for (var i = 0; i < this.n; i++) fn(this.at(i), this.at(i + 1), i);
  };

  // Aire signée (formule du lacet).
  Polygon.prototype.signedArea = function () {
    var s = 0;
    this.eachEdge(function (a, b) {
      s += a.x * b.y - b.x * a.y;
    });
    return s / 2;
  };

  Polygon.prototype.area = function () {
    return Math.abs(this.signedArea());
  };

  Polygon.prototype.perimeter = function () {
    var p = 0;
    this.eachEdge(function (a, b) {
      p += dist(a, b);
    });
    return p;
  };

  // Centre de masse de la surface (pas la moyenne des sommets).
  Polygon.prototype.centroid = function () {
    var a6 = this.signedArea() * 6;
    if (Math.abs(a6) < EPS) {
      var s = { x: 0, y: 0 };
      for (var i = 0; i < this.n; i++) s = add(s, this.points[i]);
      return mul(s, 1 / Math.max(1, this.n));
    }
    var cx = 0,
      cy = 0;
    this.eachEdge(function (a, b) {
      var w = a.x * b.y - b.x * a.y;
      cx += (a.x + b.x) * w;
      cy += (a.y + b.y) * w;
    });
    return { x: cx / a6, y: cy / a6 };
  };

  Polygon.prototype.bbox = function () {
    return bbox(this.points);
  };

  // Sens trigonométrique au sens mathématique (voir l'entête sur le repère).
  Polygon.prototype.isCCW = function () {
    return this.signedArea() > 0;
  };

  // Copie orientée dans le sens demandé.
  Polygon.prototype.oriented = function (ccw) {
    if (ccw === undefined) ccw = true;
    return this.isCCW() === ccw
      ? this.clone()
      : new Polygon(this.points.slice().reverse());
  };

  // Le sommet i est-il réflexe (angle intérieur > 180°) ?
  Polygon.prototype.isReflex = function (i) {
    var s = this.isCCW() ? 1 : -1;
    return s * cross3(this.at(i - 1), this.at(i), this.at(i + 1)) < -EPS;
  };

  Polygon.prototype.reflexVertices = function () {
    var out = [];
    for (var i = 0; i < this.n; i++) if (this.isReflex(i)) out.push(i);
    return out;
  };

  // Angle intérieur au sommet i, en radians, dans ]0, 2π[.
  Polygon.prototype.interiorAngle = function (i) {
    var u = sub(this.at(i - 1), this.at(i));
    var v = sub(this.at(i + 1), this.at(i));
    var d = len(u) * len(v);
    if (d < EPS) return 0;
    var a = Math.acos(Math.max(-1, Math.min(1, dot(u, v) / d)));
    return this.isReflex(i) ? 2 * Math.PI - a : a;
  };

  Polygon.prototype.isConvex = function () {
    var sign = 0;
    for (var i = 0; i < this.n; i++) {
      var o = orient(this.at(i - 1), this.at(i), this.at(i + 1));
      if (o === 0) continue;
      if (sign === 0) sign = o;
      else if (o !== sign) return false;
    }
    return true;
  };

  // Le bord se recoupe-t-il ?
  Polygon.prototype.isSimple = function () {
    var n = this.n;
    if (n < 3) return false;
    for (var i = 0; i < n; i++) {
      var e1 = this.edge(i);
      if (dist2(e1[0], e1[1]) < EPS) return false;
      for (var j = i + 1; j < n; j++) {
        var e2 = this.edge(j);
        var adjacent = j === i + 1 || (i === 0 && j === n - 1);
        if (adjacent) {
          // Deux arêtes voisines ne doivent partager que leur sommet commun.
          var shared = j === i + 1 ? e1[1] : e1[0];
          var other = j === i + 1 ? e2[1] : e2[0];
          if (onSegment(other, e1[0], e1[1]) && dist2(other, shared) > EPS)
            return false;
        } else if (segmentsIntersect(e1[0], e1[1], e2[0], e2[1])) {
          return false;
        }
      }
    }
    return true;
  };

  // 'inside' | 'boundary' | 'outside'
  Polygon.prototype.locate = function (p) {
    var onBorder = false;
    this.eachEdge(function (a, b) {
      if (!onBorder && onSegment(p, a, b, 1e-6)) onBorder = true;
    });
    if (onBorder) return "boundary";
    var inside = false;
    this.eachEdge(function (a, b) {
      if (a.y > p.y === b.y > p.y) return;
      var xHit = a.x + ((p.y - a.y) / (b.y - a.y)) * (b.x - a.x);
      if (p.x < xHit) inside = !inside;
    });
    return inside ? "inside" : "outside";
  };

  Polygon.prototype.contains = function (p, includeBoundary) {
    var loc = this.locate(p);
    return (
      loc === "inside" || (includeBoundary !== false && loc === "boundary")
    );
  };

  // [at(i), at(j)] est-il une diagonale interne ?
  Polygon.prototype.isDiagonal = function (i, j) {
    var n = this.n,
      a = this.at(i),
      b = this.at(j);
    if (i === j || (j - i + n) % n === 1 || (i - j + n) % n === 1) return false;
    for (var k = 0; k < n; k++) {
      if (k === i || k === j || (k + 1) % n === i || (k + 1) % n === j)
        continue;
      if (segmentsIntersect(a, b, this.at(k), this.at(k + 1))) return false;
    }
    return this.locate(mid(a, b)) === "inside";
  };

  /**
   * Triangulation par découpe d'oreilles (ear clipping)
   * @returns {Array<[number,number,number]>} triplets d'indices dans `points`
   */
  Polygon.prototype.triangulate = function () {
    var n = this.n;
    if (n < 3) return [];
    if (n === 3) return [[0, 1, 2]];

    var self = this;
    var idx = [];
    for (var i = 0; i < n; i++) idx.push(i);
    if (!this.isCCW()) idx.reverse();

    function P(k) {
      return self.points[idx[k]];
    }

    var triangles = [],
      guard = 0;
    while (idx.length > 3 && guard++ < 10 * n) {
      var clipped = false;
      for (var k = 0; k < idx.length; k++) {
        var m = idx.length;
        var ip = (k - 1 + m) % m,
          inx = (k + 1) % m;
        var a = P(ip),
          b = P(k),
          c = P(inx);
        if (orient(a, b, c) <= 0) continue;

        var ear = true;
        for (var q = 0; q < m && ear; q++) {
          if (q === ip || q === k || q === inx) continue;
          var p = P(q);
          if (
            orient(a, b, p) >= 0 &&
            orient(b, c, p) >= 0 &&
            orient(c, a, p) >= 0
          )
            ear = false;
        }
        if (!ear) continue;

        triangles.push([idx[ip], idx[k], idx[inx]]);
        idx.splice(k, 1);
        clipped = true;
        break;
      }
      if (!clipped) break; // polygone non simple : on s'arrête
    }
    if (idx.length === 3) triangles.push([idx[0], idx[1], idx[2]]);
    return triangles;
  };

  // Triangulation renvoyée comme liste de sous-polygones.
  Polygon.prototype.triangles = function () {
    var self = this;
    return this.triangulate().map(function (t) {
      return new Polygon([
        self.points[t[0]],
        self.points[t[1]],
        self.points[t[2]],
      ]);
    });
  };

  // transformations

  Polygon.prototype.clone = function () {
    return new Polygon(
      this.points.map(function (p) {
        return { x: p.x, y: p.y };
      })
    );
  };

  Polygon.prototype.map = function (fn) {
    return new Polygon(this.points.map(fn));
  };

  Polygon.prototype.translate = function (dx, dy) {
    return this.map(function (p) {
      return { x: p.x + dx, y: p.y + dy };
    });
  };

  Polygon.prototype.scaled = function (k, center) {
    var c = center || this.centroid();
    return this.map(function (p) {
      return add(c, mul(sub(p, c), k));
    });
  };

  Polygon.prototype.rotated = function (angle, center) {
    var c = center || this.centroid();
    return this.map(function (p) {
      return rotateAround(p, angle, c);
    });
  };

  // Recentre et met à l'échelle pour tenir dans une boîte width × height.
  Polygon.prototype.fitTo = function (width, height, padding) {
    padding = padding || 0;
    var b = this.bbox();
    if (!b) return this.clone();
    var k = Math.min(
      (width - 2 * padding) / (b.width || 1),
      (height - 2 * padding) / (b.height || 1)
    );
    return this.map(function (p) {
      return {
        x: padding + (p.x - b.minX) * k,
        y: padding + (p.y - b.minY) * k,
      };
    });
  };

  Polygon.prototype.toArray = function () {
    return this.points.map(function (p) {
      return [p.x, p.y];
    });
  };

  Polygon.prototype.toJSON = function () {
    return { type: "Polygon", points: this.toArray() };
  };
  Polygon.fromJSON = function (o) {
    return new Polygon(o.points);
  };

  Polygon.prototype.toString = function () {
    return "Polygon(n=" + this.n + ", aire=" + this.area().toFixed(2) + ")";
  };

  // Graph

  /**
   * Graphe non orienté (ou orienté) à listes d'adjacence.
   * Chaque sommet porte un objet `data` libre — on y range sa position {x, y}.
   */
  function Graph(options) {
    this.directed = !!(options && options.directed);
    this._vertices = new Map(); // clé -> { id, data }
    this._adj = new Map(); // clé -> Set(clés)
    this._edges = new Map(); // clé d'arête -> { u, v, data }
  }

  function K(u) {
    return String(u);
  }

  Graph.prototype._ek = function (u, v) {
    var a = K(u),
      b = K(v);
    return this.directed || a <= b ? a + "\u0000" + b : b + "\u0000" + a;
  };

  Graph.prototype.addVertex = function (id, data) {
    var k = K(id),
      v = this._vertices.get(k);
    if (v) Object.assign(v.data, data || {});
    else {
      this._vertices.set(k, { id: id, data: Object.assign({}, data) });
      this._adj.set(k, new Set());
    }
    return this;
  };

  Graph.prototype.addEdge = function (u, v, data) {
    this.addVertex(u).addVertex(v);
    var ku = K(u),
      kv = K(v);
    if (ku === kv) return this;
    this._adj.get(ku).add(kv);
    if (!this.directed) this._adj.get(kv).add(ku);
    var ek = this._ek(u, v);
    if (this._edges.has(ek))
      Object.assign(this._edges.get(ek).data, data || {});
    else this._edges.set(ek, { u: u, v: v, data: Object.assign({}, data) });
    return this;
  };

  Graph.prototype.addEdges = function (pairs) {
    for (var i = 0; i < pairs.length; i++)
      this.addEdge(pairs[i][0], pairs[i][1]);
    return this;
  };

  Graph.prototype.removeEdge = function (u, v) {
    var ku = K(u),
      kv = K(v);
    if (this._adj.has(ku)) this._adj.get(ku).delete(kv);
    if (!this.directed && this._adj.has(kv)) this._adj.get(kv).delete(ku);
    this._edges.delete(this._ek(u, v));
    return this;
  };

  Graph.prototype.removeVertex = function (id) {
    var k = K(id),
      self = this;
    if (!this._vertices.has(k)) return this;
    Array.from(this._adj.get(k)).forEach(function (o) {
      self.removeEdge(id, self._vertices.get(o).id);
    });
    this._adj.forEach(function (set, ko) {
      if (set.has(k)) self.removeEdge(self._vertices.get(ko).id, id);
    });
    this._adj.delete(k);
    this._vertices.delete(k);
    return this;
  };

  Object.defineProperty(Graph.prototype, "order", {
    get: function () {
      return this._vertices.size;
    },
  });
  Object.defineProperty(Graph.prototype, "size", {
    get: function () {
      return this._edges.size;
    },
  });

  Graph.prototype.hasVertex = function (id) {
    return this._vertices.has(K(id));
  };
  Graph.prototype.hasEdge = function (u, v) {
    return this._edges.has(this._ek(u, v));
  };

  Graph.prototype.vertices = function () {
    return Array.from(this._vertices.values()).map(function (v) {
      return v.id;
    });
  };

  Graph.prototype.edges = function () {
    return Array.from(this._edges.values());
  };

  Graph.prototype.data = function (id) {
    var v = this._vertices.get(K(id));
    return v ? v.data : undefined;
  };

  Graph.prototype.position = function (id) {
    var d = this.data(id);
    return d && typeof d.x === "number" ? { x: d.x, y: d.y } : null;
  };

  Graph.prototype.setPosition = function (id, p) {
    return this.addVertex(id, { x: p.x, y: p.y });
  };

  Graph.prototype.neighbors = function (id) {
    var set = this._adj.get(K(id)),
      self = this;
    return set
      ? Array.from(set).map(function (k) {
          return self._vertices.get(k).id;
        })
      : [];
  };

  Graph.prototype.degree = function (id) {
    var set = this._adj.get(K(id));
    return set ? set.size : 0;
  };

  // Parcours bfs
  Graph.prototype.bfs = function (start) {
    var dist = new Map([[K(start), 0]]);
    var parent = new Map([[K(start), null]]);
    var order = [start],
      queue = [start];
    for (var h = 0; h < queue.length; h++) {
      var u = queue[h],
        nb = this.neighbors(u);
      for (var i = 0; i < nb.length; i++) {
        var v = nb[i];
        if (dist.has(K(v))) continue;
        dist.set(K(v), dist.get(K(u)) + 1);
        parent.set(K(v), u);
        order.push(v);
        queue.push(v);
      }
    }
    return { order: order, dist: dist, parent: parent };
  };

  // Parcours dfs
  Graph.prototype.dfs = function (start, visit) {
    var seen = new Set(),
      order = [],
      stack = [start];
    while (stack.length) {
      var u = stack.pop();
      if (seen.has(K(u))) continue;
      seen.add(K(u));
      order.push(u);
      if (visit) visit(u);
      var nb = this.neighbors(u);
      for (var i = nb.length - 1; i >= 0; i--) stack.push(nb[i]);
    }
    return order;
  };

  // Plus court chemin en nombre d'arêtes, ou null.
  Graph.prototype.shortestPath = function (u, v) {
    var r = this.bfs(u);
    if (!r.dist.has(K(v))) return null;
    var path = [],
      cur = v;
    while (cur !== null && cur !== undefined) {
      path.push(cur);
      cur = r.parent.get(K(cur));
    }
    return path.reverse();
  };

  Graph.prototype.components = function () {
    var seen = new Set(),
      comps = [],
      vs = this.vertices();
    for (var i = 0; i < vs.length; i++) {
      if (seen.has(K(vs[i]))) continue;
      var order = this.bfs(vs[i]).order;
      order.forEach(function (x) {
        seen.add(K(x));
      });
      comps.push(order);
    }
    return comps;
  };

  Graph.prototype.isConnected = function () {
    return this.order === 0 || this.components().length === 1;
  };

  Graph.prototype.isTree = function () {
    return this.isConnected() && this.size === this.order - 1;
  };

  // Cherche un cycle ; renvoie la suite des sommets ou null.
  Graph.prototype.findCycle = function () {
    var parent = new Map(),
      seen = new Set(),
      vs = this.vertices();
    for (var r = 0; r < vs.length; r++) {
      if (seen.has(K(vs[r]))) continue;
      var stack = [[vs[r], null]];
      while (stack.length) {
        var top = stack.pop(),
          u = top[0],
          from = top[1];
        if (seen.has(K(u))) continue;
        seen.add(K(u));
        parent.set(K(u), from);
        var nb = this.neighbors(u);
        for (var i = 0; i < nb.length; i++) {
          var v = nb[i];
          if (from !== null && K(v) === K(from)) continue;
          if (seen.has(K(v))) {
            var branch = [],
              cur = u;
            while (cur !== null && K(cur) !== K(v)) {
              branch.push(cur);
              cur = parent.get(K(cur));
            }
            if (cur !== null) return [v].concat(branch.reverse());
          } else {
            stack.push([v, u]);
          }
        }
      }
    }
    return null;
  };

  // Le dessin courant est-il planaire ?
  Graph.prototype.isPlaneDrawing = function () {
    var self = this;
    var es = this.edges().filter(function (e) {
      return self.position(e.u) && self.position(e.v);
    });
    for (var i = 0; i < es.length; i++) {
      for (var j = i + 1; j < es.length; j++) {
        var a = es[i].u,
          b = es[i].v,
          c = es[j].u,
          d = es[j].v;
        if (K(a) === K(c) || K(a) === K(d) || K(b) === K(c) || K(b) === K(d))
          continue;
        if (
          segmentsIntersect(
            this.position(a),
            this.position(b),
            this.position(c),
            this.position(d)
          )
        )
          return false;
      }
    }
    return true;
  };

  // Place les sommets sur un cercle.
  Graph.prototype.layoutCircle = function (radius, center) {
    center = center || { x: 0, y: 0 };
    var vs = this.vertices(),
      self = this;
    vs.forEach(function (v, i) {
      var a = (2 * Math.PI * i) / vs.length;
      self.setPosition(v, {
        x: center.x + radius * Math.cos(a),
        y: center.y + radius * Math.sin(a),
      });
    });
    return this;
  };

  Graph.prototype.clone = function () {
    var g = new Graph({ directed: this.directed });
    this._vertices.forEach(function (v) {
      g.addVertex(v.id, Object.assign({}, v.data));
    });
    this.edges().forEach(function (e) {
      g.addEdge(e.u, e.v, Object.assign({}, e.data));
    });
    return g;
  };

  Graph.prototype.toJSON = function () {
    return {
      type: "Graph",
      directed: this.directed,
      vertices: Array.from(this._vertices.values()).map(function (v) {
        return { id: v.id, data: v.data };
      }),
      edges: this.edges().map(function (e) {
        return { u: e.u, v: e.v, data: e.data };
      }),
    };
  };

  Graph.fromJSON = function (o) {
    var g = new Graph({ directed: !!o.directed });
    (o.vertices || []).forEach(function (v) {
      g.addVertex(v.id, v.data || {});
    });
    (o.edges || []).forEach(function (e) {
      g.addEdge(e.u, e.v, e.data || {});
    });
    return g;
  };

  Graph.prototype.toString = function () {
    return "Graph(|V|=" + this.order + ", |E|=" + this.size + ")";
  };

  Graph.cycle = function (n, radius, center) {
    var g = new Graph();
    center = center || { x: 0, y: 0 };
    radius = radius || 1;
    for (var i = 0; i < n; i++) {
      var a = (2 * Math.PI * i) / n;
      g.addVertex(i, {
        x: center.x + radius * Math.cos(a),
        y: center.y + radius * Math.sin(a),
      });
    }
    for (var j = 0; j < n; j++) g.addEdge(j, (j + 1) % n);
    return g;
  };

  Graph.complete = function (n, radius, center) {
    var g = Graph.cycle(n, radius, center);
    for (var i = 0; i < n; i++) for (var j = i + 1; j < n; j++) g.addEdge(i, j);
    return g;
  };

  Graph.grid = function (cols, rows, step) {
    var g = new Graph();
    step = step || 1;
    for (var r = 0; r < rows; r++) {
      for (var c = 0; c < cols; c++) {
        g.addVertex(c + "," + r, { x: c * step, y: r * step });
        if (c > 0) g.addEdge(c - 1 + "," + r, c + "," + r);
        if (r > 0) g.addEdge(c + "," + (r - 1), c + "," + r);
      }
    }
    return g;
  };

  // Cycle du bord d'un polygone : le sommet i correspond au sommet i.
  Graph.fromPolygon = function (polygon) {
    var g = new Graph();
    polygon.points.forEach(function (p, i) {
      g.addVertex(i, { x: p.x, y: p.y });
    });
    for (var i = 0; i < polygon.n; i++) g.addEdge(i, (i + 1) % polygon.n);
    return g;
  };

  // Bord + diagonales d'une triangulation.
  Graph.fromTriangulation = function (polygon) {
    var g = Graph.fromPolygon(polygon);
    polygon.triangulate().forEach(function (t) {
      g.addEdge(t[0], t[1]);
      g.addEdge(t[1], t[2]);
      g.addEdge(t[2], t[0]);
    });
    return g;
  };

  // 4. Dessin

  function opt(o, key, fallback) {
    return o && o[key] !== undefined ? o[key] : fallback;
  }

  // Dessine un polygone fermé.
  function drawPolygon(poly, o) {
    if (!poly || poly.n === 0) return;
    var fillColor = opt(o, "fill", [70, 130, 200, 45]);
    var strokeColor = opt(o, "stroke", [43, 108, 176]);
    var weight = opt(o, "strokeWeight", 2);

    push();
    if (fillColor) fill.apply(null, [].concat(fillColor));
    else noFill();
    stroke.apply(null, [].concat(strokeColor));
    strokeWeight(weight);
    strokeJoin(ROUND);
    beginShape();
    for (var i = 0; i < poly.n; i++) vertex(poly.points[i].x, poly.points[i].y);
    endShape(CLOSE);

    if (opt(o, "showTriangulation", false)) {
      noFill();
      strokeWeight(1);
      stroke(strokeColor[0], strokeColor[1], strokeColor[2], 110);
      poly.triangulate().forEach(function (t) {
        beginShape();
        for (var k = 0; k < 3; k++)
          vertex(poly.points[t[k]].x, poly.points[t[k]].y);
        endShape(CLOSE);
      });
    }
    pop();

    if (opt(o, "vertices", true)) {
      drawVertices(poly.points, {
        radius: opt(o, "vertexRadius", 5),
        fill: strokeColor,
        labels: opt(o, "labels", false),
        reflex: opt(o, "markReflex", false) ? poly.reflexVertices() : [],
        reflexFill: opt(o, "reflexFill", [214, 69, 69]),
      });
    }
  }

  // Dessine un. polygone en cours de construction
  function drawPolyline(points, o) {
    if (!points.length) return;
    push();
    noFill();
    stroke.apply(null, [].concat(opt(o, "stroke", [43, 108, 176])));
    strokeWeight(opt(o, "strokeWeight", 2));
    beginShape();
    for (var i = 0; i < points.length; i++) vertex(points[i].x, points[i].y);
    endShape();
    pop();
    if (opt(o, "vertices", true)) {
      drawVertices(points, {
        radius: opt(o, "vertexRadius", 5),
        fill: opt(o, "stroke", [43, 108, 176]),
        labels: opt(o, "labels", false),
      });
    }
  }

  // Dessine une liste de sommets
  function drawVertices(points, o) {
    var r = opt(o, "radius", 5);
    var base = opt(o, "fill", [40, 40, 40]);
    var reflex = opt(o, "reflex", []);
    var reflexFill = opt(o, "reflexFill", [214, 69, 69]);
    var labels = opt(o, "labels", false);
    push();
    noStroke();
    for (var i = 0; i < points.length; i++) {
      var isReflex = reflex.indexOf(i) !== -1;
      fill.apply(null, [].concat(isReflex ? reflexFill : base));
      circle(points[i].x, points[i].y, r * 2);
      if (labels) {
        fill(70);
        textSize(12);
        textAlign(LEFT, BOTTOM);
        text(i, points[i].x + 8, points[i].y - 6);
      }
    }
    pop();
  }

  // Dessine un graphe
  function drawGraph(g, o) {
    var strokeColor = opt(o, "stroke", [90, 90, 90]);
    var vertexFill = opt(o, "vertexFill", [30, 30, 30]);
    var r = opt(o, "vertexRadius", 6);
    var labels = opt(o, "labels", false);
    var hl = new Set(opt(o, "highlight", []).map(String));

    push();
    stroke.apply(null, [].concat(strokeColor));
    strokeWeight(opt(o, "strokeWeight", 1.5));
    g.edges().forEach(function (e) {
      var a = g.position(e.u),
        b = g.position(e.v);
      if (a && b) line(a.x, a.y, b.x, b.y);
    });
    noStroke();
    g.vertices().forEach(function (v) {
      var p = g.position(v);
      if (!p) return;
      fill.apply(
        null,
        [].concat(hl.has(String(v)) ? [224, 138, 46] : vertexFill)
      );
      circle(p.x, p.y, r * 2);
      if (labels) {
        fill(70);
        textSize(12);
        textAlign(LEFT, BOTTOM);
        text(v, p.x + 8, p.y - 6);
      }
    });
    pop();
  }

  function drawPoint(p, o) {
    push();
    noStroke();
    fill.apply(null, [].concat(opt(o, "fill", [224, 138, 46])));
    circle(p.x, p.y, opt(o, "radius", 5) * 2);
    var label = opt(o, "label", null);
    if (label !== null) {
      fill(70);
      textSize(12);
      textAlign(LEFT, BOTTOM);
      text(label, p.x + 8, p.y - 6);
    }
    pop();
  }

  function drawSegment(a, b, o) {
    push();
    stroke.apply(null, [].concat(opt(o, "stroke", [224, 138, 46])));
    strokeWeight(opt(o, "strokeWeight", 2));
    if (opt(o, "dashed", false) && typeof drawingContext !== "undefined") {
      drawingContext.setLineDash([6, 5]);
    }
    line(a.x, a.y, b.x, b.y);
    if (opt(o, "dashed", false) && typeof drawingContext !== "undefined") {
      drawingContext.setLineDash([]);
    }
    pop();
  }

  function drawGrid(o) {
    var step = opt(o, "step", 40);
    var col = opt(o, "stroke", [0, 0, 0, 22]);
    push();
    stroke.apply(null, [].concat(col));
    strokeWeight(1);
    for (var x = 0; x <= width; x += step) line(x, 0, x, height);
    for (var y = 0; y <= height; y += step) line(0, y, width, y);
    pop();
  }

  // sélection à la souris

  // Index du point le plus proche de `p`
  function pickVertex(points, p, radius) {
    radius = radius === undefined ? 10 : radius;
    var best = null,
      bestD = radius * radius;
    for (var i = 0; i < points.length; i++) {
      var d = dist2(points[i], p);
      if (d <= bestD) {
        bestD = d;
        best = i;
      }
    }
    return best;
  }

  // Index de l'arête du polygone la plus proche de `p`, ou null
  function pickEdge(poly, p, radius) {
    radius = radius === undefined ? 8 : radius;
    var best = null,
      bestD = radius;
    for (var i = 0; i < poly.n; i++) {
      var e = poly.edge(i);
      var d = distToSegment(p, e[0], e[1]);
      if (d <= bestD) {
        bestD = d;
        best = i;
      }
    }
    return best;
  }

  // Sommet du graphe le plus proche de `p`, ou null
  function pickGraphVertex(g, p, radius) {
    radius = radius === undefined ? 12 : radius;
    var best = null,
      bestD = radius * radius;
    g.vertices().forEach(function (v) {
      var pos = g.position(v);
      if (!pos) return;
      var d = dist2(pos, p);
      if (d <= bestD) {
        bestD = d;
        best = v;
      }
    });
    return best;
  }

  // 5. Export global

  global.Geo = {
    version: "1.0.0",
    EPS: EPS,

    // vecteurs
    pt: pt,
    add: add,
    sub: sub,
    mul: mul,
    dot: dot,
    cross: cross,
    len: len,
    dist: dist,
    dist2: dist2,
    normalize: normalize,
    lerp: lerp2,
    mid: mid,
    rotateAround: rotateAround,

    // prédicats
    orient: orient,
    cross3: cross3,
    onSegment: onSegment,
    distToSegment: distToSegment,
    segmentsIntersect: segmentsIntersect,
    lineIntersection: lineIntersection,
    segmentIntersection: segmentIntersection,
    bbox: bbox,
    convexHull: convexHull,

    // structures
    Polygon: Polygon,
    Graph: Graph,

    // dessin
    drawPolygon: drawPolygon,
    drawPolyline: drawPolyline,
    drawVertices: drawVertices,
    drawGraph: drawGraph,
    drawPoint: drawPoint,
    drawSegment: drawSegment,
    drawGrid: drawGrid,

    // sélection
    pickVertex: pickVertex,
    pickEdge: pickEdge,
    pickGraphVertex: pickGraphVertex,
  };
})(typeof window !== "undefined" ? window : globalThis);
