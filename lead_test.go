package projectile

import (
	"math"
	"math/rand"
	"testing"
)

func TestHitHasPositiveTime(t *testing.T) {
	elapsed, _, ok := (&Engine{}).Lead(Target{P: Vec{X: 100}}, 100)
	if !ok || elapsed <= 0 {
		t.Fatalf("正前方的固定目标应该给出正的开火时刻：%v %v", elapsed, ok)
	}
}

func TestDirectionIsNormalized(t *testing.T) {
	_, dir, _ := (&Engine{}).Lead(Target{P: Vec{X: 100}}, 100)
	if math.Abs(math.Hypot(dir.X, dir.Y)-1) > 1e-9 {
		t.Fatalf("开火方向应该是单位向量：%+v", dir)
	}
}

func TestStepsStartAtZero(t *testing.T) {
	if (&Engine{}).Steps() != 0 {
		t.Fatal("初始求值次数应该是 0")
	}
}

func TestConstants(t *testing.T) {
	if SimStepMs != 10 || MaxFlightMs != 5000 || HitRadius != 0.5 || MaxIter != 64 {
		t.Fatal("弹道常量被改了")
	}
}

func approx(a, b, tol float64) bool { return math.Abs(a-b) <= tol }

// TestHitRadius 正前方 100 米固定靶：命中半径 0.5 米时最早时刻是 995ms 而不是 1000ms。
func TestHitRadius(t *testing.T) {
	engine := &Engine{}
	elapsed, dir, ok := engine.Lead(Target{P: Vec{X: 100}}, 100)
	if !ok {
		t.Fatal("固定靶应当可命中")
	}
	if !approx(elapsed, 995.0, 1e-3) {
		t.Fatalf("命中半径应让最早时刻为 995ms，实际 %v", elapsed)
	}
	if !approx(dir.X, 1, 1e-12) || dir.Y != 0 {
		t.Fatalf("固定靶方向应指向正前方：%+v", dir)
	}
	// 命中时刻子弹行程加半径恰好覆盖目标距离。
	flight := elapsed / 1000
	if math.Abs(100-(100*flight+HitRadius)) > 1e-6 {
		t.Fatalf("命中时刻应贴在命中半径边界上：剩余 %v", 100-(100*flight+HitRadius))
	}
}

// TestZeroTimeHit 目标零时刻已经在命中半径内时直接返回 0。
func TestZeroTimeHit(t *testing.T) {
	engine := &Engine{}
	elapsed, _, ok := engine.Lead(Target{P: Vec{X: HitRadius}}, 100)
	if !ok || elapsed != 0 {
		t.Fatalf("零时刻已在半径内应返回 0：%v %v", elapsed, ok)
	}
}

// TestAcceleratingTarget 目标以 -200m/s² 冲向射手，最早时刻约 615.8ms。
func TestAcceleratingTarget(t *testing.T) {
	engine := &Engine{}
	elapsed, _, ok := engine.Lead(Target{P: Vec{X: 100}, A: Vec{X: -200}}, 100)
	if !ok {
		t.Fatal("加速冲来的目标应当可命中")
	}
	// 解析根：100-100t² = 100t+0.5 => t = (-100+sqrt(49800))/200 秒。
	exact := (-100 + math.Sqrt(49800)) / 200 * 1000
	if !approx(elapsed, exact, 1e-3) {
		t.Fatalf("加速目标最早时刻应为 615.8ms，实际 %v", elapsed)
	}
}

// TestDirectionPointsAtFuturePosition 开火方向必须指向命中时刻的目标位置。
func TestDirectionPointsAtFuturePosition(t *testing.T) {
	target := Target{P: Vec{X: 80, Y: 60}, V: Vec{X: 30, Y: -20}, A: Vec{X: -10, Y: 40}}
	engine := &Engine{}
	elapsed, dir, ok := engine.Lead(target, 200)
	if !ok {
		t.Fatal("该目标应当可命中")
	}
	future := targetAt(target, elapsed/1000)
	want := direction(future)
	if math.Hypot(dir.X-want.X, dir.Y-want.Y) > 1e-9 {
		t.Fatalf("方向 %+v 未指向命中时刻目标位置 %+v（%+v）", dir, future, want)
	}
}

// TestNoSolution 同速向外逃逸还在向外加速：全程追不上，必须报 false。
func TestNoSolution(t *testing.T) {
	engine := &Engine{}
	elapsed, _, ok := engine.Lead(Target{
		P: Vec{X: 100},
		V: Vec{X: 100},
		A: Vec{X: 1000},
	}, 100)
	if ok {
		t.Fatalf("逃逸目标不应被判成可命中：t=%v", elapsed)
	}
	if elapsed != 0 {
		t.Fatalf("无解时时刻应为 0，实际 %v", elapsed)
	}
}

// TestNoSolutionBeyondMaxFlight 目标先逃远、在 MaxFlight 之后才慢下来：窗口外无解。
func TestNoSolutionBeyondMaxFlight(t *testing.T) {
	// 静止目标距离 600 米、弹速 100：子弹 5 秒只能飞 500 米，加半径仍够不到。
	engine := &Engine{}
	if _, _, ok := engine.Lead(Target{P: Vec{X: 600}}, 100); ok {
		t.Fatal("500 米射程加半径仍够不到 600 米目标，必须无解")
	}
}

// TestMaxFlightBoundaryFeasible 恰好能在飞行上限之前追上时应当命中。
func TestMaxFlightBoundaryFeasible(t *testing.T) {
	// 5 秒子弹飞 500 米 + 半径 0.5：目标在 500.4 米处静止，5 秒时够不到，
	// 改成 500.4 米、以 0.01m/s 朝射手移动，保证窗口落在上限之内。
	engine := &Engine{}
	elapsed, _, ok := engine.Lead(Target{P: Vec{X: 500.4}, V: Vec{X: -0.01}}, 100)
	if !ok {
		t.Fatal("上限之内出现可行窗口时应当命中")
	}
	if elapsed > MaxFlightMs || elapsed < MaxFlightMs-100 {
		t.Fatalf("命中时刻应贴近飞行上限：%v", elapsed)
	}
}

// TestStepsBudget 无论目标怎样运动，单次解算求值不得超过 MaxIter。
func TestStepsBudget(t *testing.T) {
	rng := rand.New(rand.NewSource(20240930))
	engine := &Engine{}
	cases := []Target{
		{P: Vec{X: 100}},
		{P: Vec{X: 100}, A: Vec{X: -200}},
		{P: Vec{X: 100}, V: Vec{X: 100}, A: Vec{X: 1000}},
	}
	for index := 0; index < 5000; index++ {
		cases = append(cases, Target{
			P: Vec{rng.NormFloat64() * 200, rng.NormFloat64() * 200},
			V: Vec{rng.NormFloat64() * 150, rng.NormFloat64() * 150},
			A: Vec{rng.NormFloat64() * 300, rng.NormFloat64() * 300},
		})
	}
	for index, target := range cases {
		before := engine.Steps()
		engine.Lead(target, 1+rng.Float64()*399)
		if cost := engine.Steps() - before; cost > MaxIter {
			t.Fatalf("第 %d 组解算求值 %d 次，超过 MaxIter=%d", index, cost, MaxIter)
		}
	}
}

// TestStepsSampleContract 1000 次解算的总步数规模（对照 CLI --sample steps）。
func TestStepsSampleContract(t *testing.T) {
	engine := &Engine{}
	for index := 0; index < 1000; index++ {
		engine.Lead(Target{P: Vec{X: 100}}, 100)
	}
	if engine.Steps() > 64000 {
		t.Fatalf("1000 次解算求值 %d 次，应在 64000 以内", engine.Steps())
	}
}

// interval 是闭区间。
type interval struct{ lo, hi float64 }

func addI(a, b interval) interval { return interval{a.lo + b.lo, a.hi + b.hi} }

func mulI(a, b interval) interval {
	vals := []float64{a.lo * b.lo, a.lo * b.hi, a.hi * b.lo, a.hi * b.hi}
	lo, hi := vals[0], vals[0]
	for _, v := range vals[1:] {
		lo = math.Min(lo, v)
		hi = math.Max(hi, v)
	}
	pad := 1e-12 * math.Max(1, math.Max(math.Abs(lo), math.Abs(hi)))
	return interval{lo - pad, hi + pad}
}

func numI(v float64) interval { return interval{v, v} }

// evalQBounds 用区间 Horner 严格包住 Q(t)=|T(t)|²-(v*t+r)² 在 x 上的取值。
func evalQBounds(q [5]float64, x interval) interval {
	result := numI(q[4])
	for k := 3; k >= 0; k-- {
		result = addI(mulI(result, x), numI(q[k]))
	}
	return result
}

// referenceLead 是独立参考实现：不解析驻点，直接对四次可行域做区间算术二分，
// 左优先细化到 1e-12 秒，保证不漏掉任何可行窗口。
func referenceLead(target Target, bulletSpeed float64) (float64, bool) {
	q := [5]float64{
		target.P.X*target.P.X + target.P.Y*target.P.Y - HitRadius*HitRadius,
		2*(target.P.X*target.V.X+target.P.Y*target.V.Y) - 2*bulletSpeed*HitRadius,
		target.V.X*target.V.X + target.V.Y*target.V.Y + target.P.X*target.A.X + target.P.Y*target.A.Y - bulletSpeed*bulletSpeed,
		target.V.X*target.A.X + target.V.Y*target.A.Y,
		(target.A.X*target.A.X + target.A.Y*target.A.Y) / 4,
	}
	eval := func(t float64) float64 {
		return (((q[4]*t+q[3])*t+q[2])*t+q[1])*t + q[0]
	}
	if eval(0) <= 0 {
		return 0, true
	}

	type job struct{ x interval }
	stack := []job{{interval{0, maxFlightSeconds}}}
	const minWidth = 1e-12
	for len(stack) > 0 {
		cur := stack[len(stack)-1]
		stack = stack[:len(stack)-1]
		bounds := evalQBounds(q, cur.x)
		if bounds.lo > 0 {
			continue // 整段不可行
		}
		mid := (cur.x.lo + cur.x.hi) / 2
		if bounds.hi <= 0 || cur.x.hi-cur.x.lo <= minWidth {
			// 整段可行（或已细化到极限）：左端点即候选的最早可行时刻。
			t := cur.x.lo
			if eval(t) <= 1e-9 { // 1e-9 的 Q 余量远小于 1e-3ms 的时间精度
				return t, true
			}
			continue
		}
		// 右段先入栈、左段后入栈，保证左优先。
		stack = append(stack, job{interval{mid, cur.x.hi}}, job{interval{cur.x.lo, mid}})
	}
	return 0, false
}

// TestRandomDifferential 200 组随机状态与独立的「带命中半径可行域求根」参考实现逐项比对。
func TestRandomDifferential(t *testing.T) {
	rng := rand.New(rand.NewSource(777))
	for index := 0; index < 200; index++ {
		target := Target{
			P: Vec{X: rng.Float64()*400 - 200, Y: rng.Float64()*400 - 200},
			V: Vec{X: rng.Float64()*300 - 150, Y: rng.Float64()*300 - 150},
			A: Vec{X: rng.Float64()*600 - 300, Y: rng.Float64()*600 - 300},
		}
		bulletSpeed := 1 + rng.Float64()*399

		engine := &Engine{}
		got, dir, gotOK := engine.Lead(target, bulletSpeed)
		want, wantOK := referenceLead(target, bulletSpeed)
		wantMs := want * 1000

		if gotOK != wantOK {
			t.Fatalf("第 %d 组可行性不一致：impl=%v ref=%v, target=%+v speed=%v",
				index, gotOK, wantOK, target, bulletSpeed)
		}
		if !gotOK {
			continue
		}
		if math.Abs(got-wantMs) > 1e-3 {
			t.Fatalf("第 %d 组时刻偏差 %vms（impl=%v ref=%v），target=%+v speed=%v",
				index, math.Abs(got-wantMs), got, wantMs, target, bulletSpeed)
		}
		if math.Hypot(dir.X, dir.Y) > 0 {
			wantDir := direction(targetAt(target, want))
			if math.Hypot(dir.X-wantDir.X, dir.Y-wantDir.Y) > 1e-6 {
				t.Fatalf("第 %d 组方向不一致：impl=%+v ref=%+v", index, dir, wantDir)
			}
		}
		if engine.Steps() > MaxIter {
			t.Fatalf("第 %d 组求值 %d 次，超过 MaxIter", index, engine.Steps())
		}
	}
}
