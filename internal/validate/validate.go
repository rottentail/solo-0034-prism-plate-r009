// Package validate owns structural admission and the shared compilation budget.
package validate

import (
	"fmt"
	"math"
	"prismplate/internal/geom"
	"prismplate/internal/scene"
	"regexp"
)

var identifier = regexp.MustCompile(`^[a-zA-Z][a-zA-Z0-9_-]{0,63}$`)

func ID(s string) bool         { return identifier.MatchString(s) }
func finite(x float64) bool    { return !math.IsNaN(x) && !math.IsInf(x, 0) && math.Abs(x) <= 1e6 }
func Unit(x float64) bool      { return finite(x) && x >= 0 && x <= 1 }
func Color(c scene.Color) bool { return Unit(c.R) && Unit(c.G) && Unit(c.B) && Unit(c.A) }
func path(ops []geom.Op) (geom.Path, error) {
	if len(ops) == 0 || len(ops) > 256 {
		return nil, fmt.Errorf("contour operation count must be 1..256")
	}
	open := false
	vertices := 0
	for _, o := range ops {
		n, ok := map[string]int{"M": 1, "L": 1, "Q": 2, "C": 3, "Z": 0}[o.Kind]
		if !ok || len(o.Points) != n {
			return nil, fmt.Errorf("invalid contour verb or arity")
		}
		for _, p := range o.Points {
			if !finite(p.X) || !finite(p.Y) {
				return nil, fmt.Errorf("non-finite contour")
			}
		}
		if o.Kind == "M" {
			if open {
				return nil, fmt.Errorf("close each contour before another M")
			}
			open = true
			vertices = 1
		} else {
			if !open {
				return nil, fmt.Errorf("contour must begin with M")
			}
			if o.Kind == "Z" {
				if vertices < 3 {
					return nil, fmt.Errorf("contour requires three vertices")
				}
				open = false
			} else {
				vertices++
			}
		}
	}
	if open {
		return nil, fmt.Errorf("contour must terminate with Z")
	}
	return geom.Flatten(ops), nil
}
func brush(b scene.Brush) error {
	if b.Kind == "solid" {
		if !Color(b.Color) {
			return fmt.Errorf("invalid solid color")
		}
		return nil
	}
	if b.Kind != "linear" {
		return fmt.Errorf("unknown brush")
	}
	for _, v := range []float64{b.From.X, b.From.Y, b.To.X, b.To.Y} {
		if !finite(v) {
			return fmt.Errorf("invalid gradient axis")
		}
	}
	if math.Hypot(b.To.X-b.From.X, b.To.Y-b.From.Y) < 1e-6 {
		return fmt.Errorf("degenerate gradient axis")
	}
	if len(b.Stops) < 2 || len(b.Stops) > 16 {
		return fmt.Errorf("gradient requires 2..16 stops")
	}
	prev := -1.0
	for _, s := range b.Stops {
		if !Unit(s.At) || s.At <= prev || !Color(s.Color) {
			return fmt.Errorf("invalid gradient stops")
		}
		prev = s.At
	}
	if b.Stops[0].At != 0 || prev != 1 {
		return fmt.Errorf("gradient endpoints must be 0 and 1")
	}
	return nil
}

// warpSpec admits a control grid only when the deformation it defines is a
// homeomorphism of the plane: border nodes pinned (so the identity outside
// the region joins without tears), interior nodes confined to the region (so
// material cannot escape the declared area), and positive corner Jacobians
// (bilinear det J attains its extremes at corners, so this is an exact
// fold proof for every point of every cell).
func warpSpec(w *geom.Warp) error {
	if !finite(w.Origin.X) || !finite(w.Origin.Y) || !finite(w.Step.X) || !finite(w.Step.Y) {
		return fmt.Errorf("invalid warp frame")
	}
	if w.Step.X <= 1e-9 || w.Step.Y <= 1e-9 {
		return fmt.Errorf("warp cell size must be positive")
	}
	if w.Cols < 1 || w.Cols > 16 || w.Rows < 1 || w.Rows > 16 {
		return fmt.Errorf("warp grid must be 1..16 cells per side")
	}
	if len(w.Points) != (w.Cols+1)*(w.Rows+1) {
		return fmt.Errorf("warp node count must be (cols+1)*(rows+1)")
	}
	if w.GradientSpace != "" && w.GradientSpace != "material" && w.GradientSpace != "canvas" {
		return fmt.Errorf("unknown gradient space")
	}
	maxX := w.Origin.X + float64(w.Cols)*w.Step.X
	maxY := w.Origin.Y + float64(w.Rows)*w.Step.Y
	for j := 0; j <= w.Rows; j++ {
		for i := 0; i <= w.Cols; i++ {
			p := w.Points[j*(w.Cols+1)+i]
			if !finite(p.X) || !finite(p.Y) {
				return fmt.Errorf("non-finite warp node")
			}
			if i == 0 || i == w.Cols || j == 0 || j == w.Rows {
				if r := w.Rest(i, j); math.Abs(p.X-r.X) > 1e-9 || math.Abs(p.Y-r.Y) > 1e-9 {
					return fmt.Errorf("warp boundary node (%d,%d) must stay pinned to keep the region edge seamless", i, j)
				}
				continue
			}
			if p.X < w.Origin.X-1e-9 || p.X > maxX+1e-9 || p.Y < w.Origin.Y-1e-9 || p.Y > maxY+1e-9 {
				return fmt.Errorf("warp node (%d,%d) leaves the grid region", i, j)
			}
		}
	}
	for j := 0; j < w.Rows; j++ {
		for i := 0; i < w.Cols; i++ {
			for corner, d := range w.CornerDets(i, j) {
				if d <= 1e-9 {
					return fmt.Errorf("warp folds material at cell (%d,%d) corner %d (jacobian %.3g)", i, j, corner, d)
				}
			}
		}
	}
	return nil
}

func ringArea(r geom.Contour) float64 {
	s := 0.0
	for i, a := range r {
		b := r[(i+1)%len(r)]
		s += a.X*b.Y - b.X*a.Y
	}
	return s / 2
}

// warpTopology replays the shape's own contours through the deformation and
// rejects results whose fill semantics would silently change: reversed
// orientation, collapsed rings (a cutout squeezed out of existence) or a
// changed containment relation (a hole escaping its body).
func warpTopology(before, after geom.Path) error {
	for i, ring := range before {
		a0 := ringArea(ring)
		if math.Abs(a0) < 1e-12 {
			continue
		}
		a1 := ringArea(after[i])
		if a1*a0 <= 0 {
			return fmt.Errorf("warp reverses contour %d orientation", i)
		}
		if math.Abs(a1) < 1e-9 || math.Abs(a1) < math.Abs(a0)*1e-3 {
			return fmt.Errorf("warp collapses contour %d below visible size", i)
		}
	}
	for i := range before {
		for j := range before {
			if i == j {
				continue
			}
			inside0 := geom.Path{before[j]}.Contains(before[i][0], "nonzero")
			inside1 := geom.Path{after[j]}.Contains(after[i][0], "nonzero")
			if inside0 != inside1 {
				return fmt.Errorf("warp moves contour %d across contour %d", i, j)
			}
		}
	}
	return nil
}
func Compile(d scene.Document) (scene.Compiled, error) {
	c := scene.Compiled{Document: d}
	if d.Width < 1 || d.Height < 1 || d.Width > 512 || d.Height > 512 || !Color(d.Background) {
		return c, fmt.Errorf("invalid canvas (maximum 512 square)")
	}
	if len(d.Layers) < 1 || len(d.Layers) > 16 {
		return c, fmt.Errorf("requires 1..16 layers")
	}
	ids := map[string]bool{}
	add := func(id string) error {
		if !ID(id) || ids[id] {
			return fmt.Errorf("invalid or duplicate node id %q", id)
		}
		ids[id] = true
		return nil
	}
	count := func(p geom.Path) {
		for _, r := range p {
			c.Segments += len(r)
		}
	}
	for _, l := range d.Layers {
		if err := add(l.ID); err != nil {
			return c, err
		}
		if !Unit(l.Opacity) || (l.Blend != "normal" && l.Blend != "multiply" && l.Blend != "screen") {
			return c, fmt.Errorf("invalid layer compositing")
		}
		if len(l.Shapes) < 1 || len(l.Shapes) > 32 {
			return c, fmt.Errorf("requires 1..32 shapes per layer")
		}
		cl := scene.CompiledLayer{Layer: l}
		if len(l.Clip) > 0 {
			p, e := path(l.Clip)
			if e != nil {
				return c, e
			}
			cl.Clip = p
			count(p)
		}
		if len(l.Mask) > 0 {
			p, e := path(l.Mask)
			if e != nil {
				return c, e
			}
			cl.Mask = p
			count(p)
		}
		for _, s := range l.Shapes {
			if err := add(s.ID); err != nil {
				return c, err
			}
			if s.FillRule != "nonzero" && s.FillRule != "evenodd" {
				return c, fmt.Errorf("invalid fill rule")
			}
			for _, v := range s.Transform {
				if !finite(v) {
					return c, fmt.Errorf("invalid transform")
				}
			}
			inv, ok := s.Transform.Inverse()
			if !ok {
				return c, fmt.Errorf("singular transform")
			}
			for _, v := range inv {
				if !finite(v) {
					return c, fmt.Errorf("ill-conditioned transform")
				}
			}
			p, e := path(s.Path)
			if e != nil {
				return c, e
			}
			if e = brush(s.Brush); e != nil {
				return c, e
			}
			if s.Warp != nil {
				if e = warpSpec(s.Warp); e != nil {
					return c, e
				}
				warped, we := s.Warp.ApplyPath(p)
				if we != nil {
					return c, we
				}
				if e = warpTopology(p, warped); e != nil {
					return c, e
				}
				p = warped
				// Material gradients pay an inverse-warp walk per covered
				// sample; bill four edge units per grid cell.
				if s.Brush.Kind == "linear" && s.Warp.MaterialGradient() {
					c.Segments += 4 * s.Warp.Cols * s.Warp.Rows
				}
			}
			count(p)
			cl.Shapes = append(cl.Shapes, scene.CompiledShape{Shape: s, Path: p, Inverse: inv})
		}
		c.Layers = append(c.Layers, cl)
	}
	// Four samples per pixel; every fill is bounded even under adversarial input.
	if int64(c.Segments)*int64(d.Width)*int64(d.Height)*4 > 160_000_000 {
		return c, fmt.Errorf("scene exceeds bounded rendering work")
	}
	return c, nil
}
