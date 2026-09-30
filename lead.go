package projectile

import "math"

// SimStepMs 是帧步长（毫秒），仅作口径常量；解算不按帧模拟。
const SimStepMs = 10.0

// MaxFlightMs 是允许的最长飞行时间（毫秒）。
const MaxFlightMs = 5000.0

// HitRadius 是命中半径（米）。
const HitRadius = 0.5

// MaxIter 是一次解算允许的可行域求值次数上限。
const MaxIter = 64

// Vec 是二维坐标。
type Vec struct {
	X, Y float64
}

// Target 是目标状态：位置、速度与加速度。
type Target struct {
	P, V, A Vec
}

// Engine 是弹道解算器。
type Engine struct {
	steps int
}

// Steps 是累计的可行域求值次数（规模观测）。
func (e *Engine) Steps() int { return e.steps }

// Lead 返回最早的可行开火时刻（毫秒）与开火方向。
//
// 目标在命中时刻 t（秒）位于 Q(t) = P + V*t + A*t²/2，子弹自原点以固定速度
// 沿直线飞行。因为开火方向可取任意单位向量，命中条件（相距不超过 HitRadius）
// 等价于 |Q(t)| - bulletSpeed*t <= HitRadius，即
// F(t) = |Q(t)|² - (bulletSpeed*t + HitRadius)² <= 0。
// F 是关于 t 的四次多项式，在其临界点（三次方程的实根）之间单调。解析求出
// 临界点后按时间顺序逐段二分最早的可行根，求值次数有界（<= MaxIter）。
func (e *Engine) Lead(target Target, bulletSpeed float64) (float64, Vec, bool) {
	// F(t) = c0 + c1*t + c2*t² + c3*t³ + c4*t⁴
	c0 := target.P.X*target.P.X + target.P.Y*target.P.Y - HitRadius*HitRadius
	c1 := 2*(target.P.X*target.V.X+target.P.Y*target.V.Y) - 2*bulletSpeed*HitRadius
	c2 := target.V.X*target.V.X + target.V.Y*target.V.Y +
		target.P.X*target.A.X + target.P.Y*target.A.Y - bulletSpeed*bulletSpeed
	c3 := target.V.X*target.A.X + target.V.Y*target.A.Y
	c4 := 0.25 * (target.A.X*target.A.X + target.A.Y*target.A.Y)

	f := func(t float64) float64 {
		e.steps++
		return ((c4*t+c3)*t+c2)*t*t + c1*t + c0
	}

	tMax := MaxFlightMs / 1000.0
	if f(0) <= 0 {
		return 0, direction(target.P), true
	}

	// F'(t) = c1 + 2 c2 t + 3 c3 t² + 4 c4 t³：用临界点把 [0, tMax] 切成
	// F 单调的小区间，每段至多一个根，切点也能被端点扫描覆盖。
	bounds := []float64{0}
	for _, root := range cubicRoots(4*c4, 3*c3, 2*c2, c1) {
		if root > 1e-12 && root < tMax-1e-12 {
			bounds = append(bounds, root)
		}
	}
	bounds = append(bounds, tMax)

	for index := 0; index+1 < len(bounds); index++ {
		lo, hi := bounds[index], bounds[index+1]
		fLo := f(lo)
		if fLo <= 0 {
			return hit(lo, target)
		}
		fHi := f(hi)
		if fHi <= 0 {
			// lo 不可行、hi 可行，二分最早的穿越点。45 次远在 MaxIter 之内，
			// 且精度远超 1e-3 毫秒的要求。
			const iterations = 45
			for step := 0; step < iterations; step++ {
				mid := (lo + hi) * 0.5
				if f(mid) <= 0 {
					hi = mid
				} else {
					lo = mid
				}
			}
			return hit(hi, target)
		}
	}
	return 0, Vec{}, false
}

func hit(tSeconds float64, target Target) (float64, Vec, bool) {
	position := targetPosition(target, tSeconds)
	return tSeconds * 1000.0, direction(position), true
}

func targetPosition(target Target, t float64) Vec {
	tt := t * t * 0.5
	return Vec{
		X: target.P.X + target.V.X*t + target.A.X*tt,
		Y: target.P.Y + target.V.Y*t + target.A.Y*tt,
	}
}

// cubicRoots 返回实系数三次（或退化的更低次）多项式 a t³+b t²+c t+d 的全部实根，
// 已排序去重。纯解析计算，不属于可行域求值。
func cubicRoots(a, b, c, d float64) []float64 {
	const eps = 1e-12
	if math.Abs(a) <= eps*math.Max(math.Abs(b), math.Max(math.Abs(c), math.Abs(d))) {
		return quadraticRoots(b, c, d)
	}
	// 化为 depressed cubic：x = t + b/(3a)，x³ + p x + q = 0。
	p := (c/a - b*b/(3*a*a)) / 3
	q := (2*b*b*b/(27*a*a*a) - b*c/(3*a*a) + d/a) / 2
	shift := b / (3 * a)
	var roots []float64

	discriminant := q*q + p*p*p
	switch {
	case discriminant > 0:
		sqrtDisc := math.Sqrt(discriminant)
		u := math.Cbrt(-q + sqrtDisc)
		v := math.Cbrt(-q - sqrtDisc)
		roots = []float64{u + v - shift}
	case discriminant < 0:
		r := math.Sqrt(-p)
		angle := math.Acos(math.Max(-1, math.Min(1, -q/(r*r*r))))
		for k := 0; k < 3; k++ {
			roots = append(roots, 2*r*math.Cos((angle+2*float64(k)*math.Pi)/3)-shift)
		}
	default:
		x := math.Cbrt(-q)
		roots = []float64{2*x - shift, -x - shift}
	}
	return dedupSorted(roots)
}

// quadraticRoots 返回 a t²+b t+c 的全部实根，退化时按一次/常数方程处理。
func quadraticRoots(a, b, c float64) []float64 {
	const eps = 1e-12
	if math.Abs(a) <= eps*math.Max(math.Abs(b), math.Abs(c)) {
		if math.Abs(b) <= eps*math.Abs(c) {
			return nil
		}
		return []float64{-c / b}
	}
	discriminant := b*b - 4*a*c
	if discriminant < 0 {
		return nil
	}
	if discriminant == 0 {
		return []float64{-b / (2 * a)}
	}
	sqrtDisc := math.Sqrt(discriminant)
	return dedupSorted([]float64{(-b - sqrtDisc) / (2 * a), (-b + sqrtDisc) / (2 * a)})
}

func dedupSorted(values []float64) []float64 {
	sortFloat64s(values)
	deduped := values[:0]
	for _, value := range values {
		if len(deduped) == 0 || math.Abs(value-deduped[len(deduped)-1]) > 1e-9 {
			deduped = append(deduped, value)
		}
	}
	return deduped
}

func sortFloat64s(values []float64) {
	// 三次方程至多三个根，插入排序即可，避免引入 sort 包的额外开销。
	for i := 1; i < len(values); i++ {
		for j := i; j > 0 && values[j-1] > values[j]; j-- {
			values[j-1], values[j] = values[j], values[j-1]
		}
	}
}

func direction(vector Vec) Vec {
	length := math.Hypot(vector.X, vector.Y)
	if length == 0 {
		return Vec{}
	}
	return Vec{X: vector.X / length, Y: vector.Y / length}
}
