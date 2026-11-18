package geom

import (
	"errors"
	"math"
	"sort"
)

// Warp is a control-grid local deformation in shape-local coordinates. The
// grid covers the rectangle Origin..Origin+Step*(Cols,Rows). Points holds
// (Rows+1)*(Cols+1) displaced node positions in row-major order: node (i,j)
// rests at Origin+(i*Step.X, j*Step.Y) and moves to Points[j*(Cols+1)+i].
// Inside each cell the map is bilinear; outside the rectangle it is the
// identity. Admission requires border nodes pinned, interior nodes inside the
// rectangle and positive corner Jacobians; together those make the map a
// homeomorphism of the plane, so seams cannot tear and containment cannot
// flip. GradientSpace selects paint sampling: "material" (default) glues the
// gradient to the deforming sheet, "canvas" keeps it fixed in place.
type Warp struct {
	Origin        Point   `json:"origin"`
	Step          Point   `json:"step"`
	Cols          int     `json:"cols"`
	Rows          int     `json:"rows"`
	Points        []Point `json:"points"`
	GradientSpace string  `json:"gradient_space,omitempty"`
}

// MaterialGradient reports whether paint follows the deformed material.
func (w *Warp) MaterialGradient() bool { return w.GradientSpace != "canvas" }

func (w *Warp) node(i, j int) Point { return w.Points[j*(w.Cols+1)+i] }

// Rest returns the undeformed position of node (i,j).
func (w *Warp) Rest(i, j int) Point {
	return Point{w.Origin.X + float64(i)*w.Step.X, w.Origin.Y + float64(j)*w.Step.Y}
}

// covers reports whether p lies inside the closed grid rectangle.
func (w *Warp) covers(p Point) bool {
	return p.X >= w.Origin.X && p.X <= w.Origin.X+float64(w.Cols)*w.Step.X &&
		p.Y >= w.Origin.Y && p.Y <= w.Origin.Y+float64(w.Rows)*w.Step.Y
}

// cell locates the rest cell containing p and its local coordinates. Callers
// guarantee p is inside the rectangle; clamps only absorb float dust.
func (w *Warp) cell(p Point) (ci, cj int, u, v float64) {
	fu := (p.X - w.Origin.X) / w.Step.X
	fv := (p.Y - w.Origin.Y) / w.Step.Y
	ci = int(math.Floor(fu))
	cj = int(math.Floor(fv))
	if ci < 0 {
		ci = 0
	} else if ci > w.Cols-1 {
		ci = w.Cols - 1
	}
	if cj < 0 {
		cj = 0
	} else if cj > w.Rows-1 {
		cj = w.Rows - 1
	}
	u = math.Max(0, math.Min(1, fu-float64(ci)))
	v = math.Max(0, math.Min(1, fv-float64(cj)))
	return ci, cj, u, v
}

// Apply maps a rest position to its deformed position.
func (w *Warp) Apply(p Point) Point {
	if !w.covers(p) {
		return p
	}
	ci, cj, u, v := w.cell(p)
	p00 := w.node(ci, cj)
	p10 := w.node(ci+1, cj)
	p01 := w.node(ci, cj+1)
	p11 := w.node(ci+1, cj+1)
	return Point{
		(1-u)*(1-v)*p00.X + u*(1-v)*p10.X + (1-u)*v*p01.X + u*v*p11.X,
		(1-u)*(1-v)*p00.Y + u*(1-v)*p10.Y + (1-u)*v*p01.Y + u*v*p11.Y,
	}
}

// MaxSubdivVertices bounds the vertices one warped path may gain when long
// edges are split at grid lines and refined inside the region.
const MaxSubdivVertices = 65536

// ApplyPath warps an already flattened path. Edges are split at grid-line
// crossings and refined inside the region so straight segments follow the
// deformation instead of cutting across it; every emitted vertex is then
// mapped by Apply. PNG and SVG share this exact output.
func (w *Warp) ApplyPath(path Path) (Path, error) {
	limit := w.Step.X
	if w.Step.Y < limit {
		limit = w.Step.Y
	}
	limit /= 2
	var out Path
	total := 0
	for _, ring := range path {
		var pts []Point
		for i, a := range ring {
			b := ring[(i+1)%len(ring)]
			pts = append(pts, w.splitEdge(a, b, limit)...)
		}
		total += len(pts)
		if total > MaxSubdivVertices {
			return nil, errSubdiv
		}
		warped := make(Contour, len(pts))
		for i, p := range pts {
			warped[i] = w.Apply(p)
		}
		out = append(out, warped)
	}
	return out, nil
}

var errSubdiv = errors.New("warp subdivision exceeds vertex budget")

// splitEdge cuts the rest-space edge a->b at grid lines and refines the parts
// lying inside the region until each is shorter than limit.
func (w *Warp) splitEdge(a, b Point, limit float64) []Point {
	ts := []float64{0, 1}
	if b.X != a.X {
		for i := 0; i <= w.Cols; i++ {
			x := w.Origin.X + float64(i)*w.Step.X
			if t := (x - a.X) / (b.X - a.X); t > 1e-12 && t < 1-1e-12 {
				ts = append(ts, t)
			}
		}
	}
	if b.Y != a.Y {
		for j := 0; j <= w.Rows; j++ {
			y := w.Origin.Y + float64(j)*w.Step.Y
			if t := (y - a.Y) / (b.Y - a.Y); t > 1e-12 && t < 1-1e-12 {
				ts = append(ts, t)
			}
		}
	}
	sort.Float64s(ts)
	at := func(t float64) Point { return Point{a.X + t*(b.X-a.X), a.Y + t*(b.Y-a.Y)} }
	var out []Point
	for i := 0; i+1 < len(ts); i++ {
		out = append(out, w.refine(at(ts[i]), at(ts[i+1]), limit)...)
	}
	return out
}

// refine emits p0 and, while the segment lies inside the region and exceeds
// limit, midpoint splits. The endpoint belongs to the next segment.
func (w *Warp) refine(p0, p1 Point, limit float64) []Point {
	if !w.covers(p0) || !w.covers(p1) {
		return []Point{p0}
	}
	if math.Hypot(p1.X-p0.X, p1.Y-p0.Y) <= limit {
		return []Point{p0}
	}
	mid := Point{(p0.X + p1.X) / 2, (p0.Y + p1.Y) / 2}
	return append(w.refine(p0, mid, limit), w.refine(mid, p1, limit)...)
}

// CornerDets returns the signed area ratios (det J normalized by the rest
// cell area) at the corners of cell (ci,cj) in order (0,0),(1,0),(0,1),(1,1).
// det J of a bilinear map is bilinear, so its extremes over the cell occur at
// corners: four positive corners prove the whole cell is fold-free.
func (w *Warp) CornerDets(ci, cj int) [4]float64 {
	p00 := w.node(ci, cj)
	p10 := w.node(ci+1, cj)
	p01 := w.node(ci, cj+1)
	p11 := w.node(ci+1, cj+1)
	a := Point{p10.X - p00.X, p10.Y - p00.Y}
	b := Point{p01.X - p00.X, p01.Y - p00.Y}
	c := Point{p11.X - p10.X - p01.X + p00.X, p11.Y - p10.Y - p01.Y + p00.Y}
	ac := Point{a.X + c.X, a.Y + c.Y}
	bc := Point{b.X + c.X, b.Y + c.Y}
	cross := func(p, q Point) float64 { return p.X*q.Y - p.Y*q.X }
	den := w.Step.X * w.Step.Y
	return [4]float64{cross(a, b) / den, cross(a, bc) / den, cross(ac, b) / den, cross(ac, bc) / den}
}

// quad is one cell's displaced corners in (0,0),(1,0),(1,1),(0,1) order.
type quad [4]Point

func (w *Warp) quad(ci, cj int) quad {
	return quad{w.node(ci, cj), w.node(ci+1, cj), w.node(ci+1, cj+1), w.node(ci, cj+1)}
}

// margin returns the worst signed edge distance of p to the convex quad; a
// non-negative value means p is inside. Validated warps keep quads convex
// with positive orientation.
func (q quad) margin(p Point) float64 {
	worst := math.Inf(1)
	for e := 0; e < 4; e++ {
		a, b := q[e], q[(e+1)%4]
		if c := (b.X-a.X)*(p.Y-a.Y) - (b.Y-a.Y)*(p.X-a.X); c < worst {
			worst = c
		}
	}
	return worst
}

// locate finds the cell whose image contains p. It walks across the violated
// edge from the rest cell, then falls back to a full scan and finally to the
// least-violating cell, so the result is always defined.
func (w *Warp) locate(p Point) (int, int) {
	ci, cj, _, _ := w.cell(p)
	bi, bj, best := ci, cj, math.Inf(-1)
	for step := 0; step <= w.Cols*w.Rows; step++ {
		q := w.quad(ci, cj)
		m := q.margin(p)
		if m > best {
			best, bi, bj = m, ci, cj
		}
		if m >= 0 {
			return ci, cj
		}
		worstEdge := 0
		worst := math.Inf(1)
		for e := 0; e < 4; e++ {
			a, b := q[e], q[(e+1)%4]
			if c := (b.X-a.X)*(p.Y-a.Y) - (b.Y-a.Y)*(p.X-a.X); c < worst {
				worst, worstEdge = c, e
			}
		}
		switch worstEdge {
		case 0:
			cj--
		case 1:
			ci++
		case 2:
			cj++
		case 3:
			ci--
		}
		if ci < 0 || ci >= w.Cols || cj < 0 || cj >= w.Rows {
			break
		}
	}
	for j := 0; j < w.Rows; j++ {
		for i := 0; i < w.Cols; i++ {
			if m := w.quad(i, j).margin(p); m >= 0 {
				return i, j
			} else if m > best {
				best, bi, bj = m, i, j
			}
		}
	}
	return bi, bj
}

// invertCell solves the bilinear map of cell (ci,cj) for the rest-local
// coordinates of p, clamped to the cell.
func (w *Warp) invertCell(ci, cj int, p Point) (float64, float64) {
	q := w.quad(ci, cj)
	a := Point{q[1].X - q[0].X, q[1].Y - q[0].Y}
	b := Point{q[3].X - q[0].X, q[3].Y - q[0].Y}
	c := Point{q[2].X - q[1].X - q[3].X + q[0].X, q[2].Y - q[1].Y - q[3].Y + q[0].Y}
	d := Point{p.X - q[0].X, p.Y - q[0].Y}
	a2 := a.Y*c.X - a.X*c.Y
	a1 := d.X*c.Y - d.Y*c.X + a.Y*b.X - a.X*b.Y
	a0 := d.X*b.Y - d.Y*b.X
	var roots []float64
	if math.Abs(a2) < 1e-18 {
		if math.Abs(a1) > 1e-18 {
			roots = append(roots, -a0/a1)
		}
	} else {
		disc := a1*a1 - 4*a2*a0
		if disc >= 0 {
			s := math.Sqrt(disc)
			roots = append(roots, (-a1+s)/(2*a2), (-a1-s)/(2*a2))
		}
	}
	u := 0.0
	found := false
	for _, r := range roots {
		if r >= -1e-9 && r <= 1+1e-9 {
			u = r
			found = true
			break
		}
	}
	if !found {
		u = 0.5
	}
	u = math.Max(0, math.Min(1, u))
	// Recover v from the better-conditioned projection.
	dx := b.X + c.X*u
	dy := b.Y + c.Y*u
	var v float64
	if math.Abs(dx) >= math.Abs(dy) && math.Abs(dx) > 1e-18 {
		v = (d.X - a.X*u) / dx
	} else if math.Abs(dy) > 1e-18 {
		v = (d.Y - a.Y*u) / dy
	}
	v = math.Max(0, math.Min(1, v))
	return u, v
}

// Invert returns the rest position whose deformed image is p. It is used for
// material-space paint sampling; callers rely on admission having proven the
// warp a homeomorphism, and the final clamp keeps float dust inside the cell.
func (w *Warp) Invert(p Point) Point {
	if !w.covers(p) {
		return p
	}
	ci, cj := w.locate(p)
	u, v := w.invertCell(ci, cj, p)
	return Point{w.Origin.X + (float64(ci)+u)*w.Step.X, w.Origin.Y + (float64(cj)+v)*w.Step.Y}
}
