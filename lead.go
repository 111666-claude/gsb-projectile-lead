// Package projectile 是弹道预判：目标带加速度运动，要在给定弹速与命中半径下找出最早的可行开火时刻。
//
// 命中条件（t 以秒计）：目标位置为 P + V*t + A*t²/2，子弹从原点以固定速度 bulletSpeed
// 直线飞行，因此 t 秒可行当且仅当
//
//	||P + V*t + A*t²/2|| <= bulletSpeed*t + HitRadius。
//
// 两边平方后得到多项式可行域 Q(t) <= 0，Q 是四次多项式。实现先解析求出 Q 的全部驻点
// （导数是三次多项式），按驻点把 [0, MaxFlightMs/1000] 切成单调区间，从左到右找到
// 第一个可行区间并用二分把端点（最早可行时刻）收敛到远优于 1e-6 秒的精度。可行域求值
// （Steps）次数恒定，与飞行时间无关，满足单次解算不超过 MaxIter 次的预算。
package projectile

import "math"

// SimStepMs 是模拟步长（毫秒，口径常量）。
const SimStepMs = 10.0

// MaxFlightMs 是允许的最长飞行时间（毫秒）。
const MaxFlightMs = 5000.0

// HitRadius 是命中半径（米）。
const HitRadius = 0.5

// MaxIter 是一次解算允许的可行域求值次数上限。
const MaxIter = 64

// maxFlightSeconds 是可行域的右端点（秒）。
const maxFlightSeconds = MaxFlightMs / 1000.0

// rootToleranceSeconds 是最早可行时刻的二分收敛阈值（秒），
// 远紧于 README 要求的 1e-3 毫秒（1e-6 秒）。
const rootToleranceSeconds = 1e-10

// bisectionIterations 是每个跨零区间的二分次数。
const bisectionIterations = 40

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

// Lead 返回最早的可行命中时刻（毫秒）与开火方向。
// 方向指向命中时刻的目标位置；t=0 时目标已经在命中半径内则直接返回 0。
// [0, MaxFlightMs] 内不存在可行时刻时 ok 为 false。
func (e *Engine) Lead(target Target, bulletSpeed float64) (float64, Vec, bool) {
	if bulletSpeed < 0 {
		bulletSpeed = 0
	}

	// Q(t) = |T(t)|^2 - (bulletSpeed*t + HitRadius)^2，按幂次存放系数。
	// T(t) = P + V*t + A*t^2/2。
	q0 := target.P.X*target.P.X + target.P.Y*target.P.Y - HitRadius*HitRadius
	q1 := 2*(target.P.X*target.V.X+target.P.Y*target.V.Y) - 2*bulletSpeed*HitRadius
	q2 := target.V.X*target.V.X + target.V.Y*target.V.Y +
		target.P.X*target.A.X + target.P.Y*target.A.Y - bulletSpeed*bulletSpeed
	q3 := target.V.X*target.A.X + target.V.Y*target.A.Y
	q4 := (target.A.X*target.A.X + target.A.Y*target.A.Y) / 4

	eval := func(t float64) float64 {
		e.steps++
		return (((q4*t+q3)*t+q2)*t+q1)*t + q0
	}

	if eval(0) <= 0 {
		return 0, direction(target.P), true
	}

	// Q'(t) = q1 + 2*q2*t + 3*q3*t^2 + 4*q4*t^3，解析求出全部驻点。
	criticals := realRootsCubic(4*q4, 3*q3, 2*q2, q1)
	cuts := []float64{0}
	for _, root := range criticals {
		if root > 0 && root < maxFlightSeconds {
			cuts = append(cuts, root)
		}
	}
	cuts = append(cuts, maxFlightSeconds)

	// 驻点之间 Q 单调；从左到右找到第一个由正变非正的跨零点。
	left := 0.0
	for _, right := range cuts[1:] {
		if right-left < 1e-9 {
			continue
		}
		if eval(right) <= 0 {
			// 最早可行时刻落在 (left, right]，二分该跨零点。
			lo, hi := left, right
			for iteration := 0; iteration < bisectionIterations; iteration++ {
				mid := (lo + hi) / 2
				if eval(mid) <= 0 {
					hi = mid
				} else {
					lo = mid
				}
				if hi-lo < rootToleranceSeconds {
					break
				}
			}
			t := hi
			return t * 1000, direction(targetAt(target, t)), true
		}
		left = right
	}

	return 0, Vec{}, false
}

// targetAt 返回目标在时刻 t（秒）的位置。
func targetAt(target Target, t float64) Vec {
	half := 0.5 * t * t
	return Vec{
		X: target.P.X + target.V.X*t + target.A.X*half,
		Y: target.P.Y + target.V.Y*t + target.A.Y*half,
	}
}

func direction(vector Vec) Vec {
	length := math.Hypot(vector.X, vector.Y)
	if length == 0 {
		return Vec{}
	}
	return Vec{X: vector.X / length, Y: vector.Y / length}
}

// realRootsCubic 解析求 a*t^3 + b*t^2 + c*t + d = 0 的全部不同实根（顺序任意）。
// 系数退化时自动降为二次/一次方程；无解时返回 nil。
func realRootsCubic(a, b, c, d float64) []float64 {
	scale := math.Max(math.Abs(a), math.Max(math.Abs(b), math.Max(math.Abs(c), math.Abs(d))))
	if scale == 0 {
		return nil
	}
	epsilon := 1e-14 * scale
	if math.Abs(a) <= epsilon {
		return realRootsQuadratic(b, c, d, epsilon)
	}

	// 归一化为 depressed cubic：x^3 + p*x + q = 0，t = x - b/(3a)。
	shift := b / (3 * a)
	p := (3*a*c - b*b) / (3 * a * a)
	q := (2*b*b*b - 9*a*b*c + 27*a*a*d) / (27 * a * a * a)
	discriminant := 4*p*p*p + 27*q*q

	var xRoots []float64
	if discriminant > epsilon {
		sqrtDiscriminant := math.Sqrt(discriminant / 108)
		base := -q / 2
		u := math.Cbrt(base + sqrtDiscriminant)
		v := math.Cbrt(base - sqrtDiscriminant)
		xRoots = []float64{u + v}
	} else if discriminant < -epsilon {
		// 三个实根：x = 2*sqrt(-p/3) * cos(theta/3 - 2*pi*k/3)。
		radius := 2 * math.Sqrt(-p/3)
		angle := math.Acos((3*q)/(p*radius)) / 3
		for k := 0; k < 3; k++ {
			xRoots = append(xRoots, radius*math.Cos(angle-2*math.Pi*float64(k)/3))
		}
	} else {
		// 重根情形（含判别式接近 0 的数值退化）：x1 = 3q/p，x2 = -3q/(2p)。
		if math.Abs(p) <= epsilon {
			xRoots = []float64{0}
		} else {
			xRoots = []float64{3 * q / p, -1.5 * q / p}
		}
	}

	roots := make([]float64, 0, len(xRoots))
	for _, x := range xRoots {
		root := x - shift
		if len(roots) == 0 || math.Abs(roots[len(roots)-1]-root) > 1e-10 {
			roots = append(roots, root)
		}
	}
	return roots
}

// realRootsQuadratic 求 b*t^2 + c*t + d = 0 的实根；scale 是系数量级，用于判定退化。
func realRootsQuadratic(b, c, d, scale float64) []float64 {
	epsilon := 1e-14 * scale
	if math.Abs(b) <= epsilon {
		if math.Abs(c) <= epsilon {
			return nil
		}
		return []float64{-d / c}
	}
	discriminant := c*c - 4*b*d
	if discriminant < -epsilon {
		return nil
	}
	if discriminant <= epsilon {
		return []float64{-c / (2 * b)}
	}
	sqrtDiscriminant := math.Sqrt(discriminant)
	return []float64{(-c - sqrtDiscriminant) / (2 * b), (-c + sqrtDiscriminant) / (2 * b)}
}
