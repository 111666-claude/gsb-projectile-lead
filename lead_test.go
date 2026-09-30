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

func approxEqual(a, b, tolerance float64) bool {
	return math.Abs(a-b) <= tolerance
}

func approxVec(a, b Vec, tolerance float64) bool {
	return approxEqual(a.X, b.X, tolerance) && approxEqual(a.Y, b.Y, tolerance)
}

// gapAt 返回目标在 t 秒时与子弹可达范围的余量：<=0 即可命中。
func gapAt(target Target, bulletSpeed, tSeconds float64) float64 {
	position := hitPosition(target, tSeconds)
	return math.Hypot(position.X, position.Y) - bulletSpeed*tSeconds - HitRadius
}

func hitPosition(target Target, tSeconds float64) Vec {
	tt := tSeconds * tSeconds * 0.5
	return Vec{
		X: target.P.X + target.V.X*tSeconds + target.A.X*tt,
		Y: target.P.Y + target.V.Y*tSeconds + target.A.Y*tt,
	}
}

// assertLeadGeometry 校验返回时刻在可行域边界、它是最早可行点、方向指向命中位置。
func assertLeadGeometry(t *testing.T, target Target, bulletSpeed, tMs float64, dir Vec) {
	t.Helper()
	tSeconds := tMs / 1000
	if gap := gapAt(target, bulletSpeed, tSeconds); gap > 1e-7 {
		t.Fatalf("返回时刻不在命中半径内：gap=%g", gap)
	}
	if tMs > 0 {
		earlier := math.Max(0, tSeconds-2e-5)
		if gap := gapAt(target, bulletSpeed, earlier); gap < -1e-9 {
			t.Fatalf("返回时刻不是最早可行时刻：更早处 gap=%g", gap)
		}
	}
	want := direction(hitPosition(target, tSeconds))
	if !approxVec(dir, want, 1e-9) {
		t.Fatalf("开火方向应指向命中时刻的目标位置：got=%+v want=%+v", dir, want)
	}
}

func TestHitRadiusShortensFlight(t *testing.T) {
	// 固定靶在 100.4 米：打要点要 1004ms，带上 0.5 米半径最早 999ms。
	target := Target{P: Vec{X: 100.4}}
	engine := &Engine{}
	elapsed, dir, ok := engine.Lead(target, 100)
	if !ok || !approxEqual(elapsed, 999.0, 1e-3) {
		t.Fatalf("命中半径应把最早时刻提前到 999ms：%v %v", elapsed, ok)
	}
	if engine.Steps() > MaxIter {
		t.Fatalf("单次解算求值 %d 次，超过 MaxIter=%d", engine.Steps(), MaxIter)
	}
	assertLeadGeometry(t, target, 100, elapsed, dir)
}

func TestAcceleratingTarget(t *testing.T) {
	// 目标以 200m/s² 冲向射手，最早时刻由 100-100t²-100t=0.5 解出（t 以秒计）。
	target := Target{P: Vec{X: 100}, A: Vec{X: -200}}
	wantSeconds := (math.Sqrt(4.98) - 1) / 2

	engine := &Engine{}
	elapsed, dir, ok := engine.Lead(target, 100)
	if !ok || !approxEqual(elapsed, wantSeconds*1000, 1e-3) {
		t.Fatalf("加速目标的最早时刻应是 %vms：%v %v", wantSeconds*1000, elapsed, ok)
	}
	if engine.Steps() > MaxIter {
		t.Fatalf("单次解算求值 %d 次，超过 MaxIter=%d", engine.Steps(), MaxIter)
	}
	assertLeadGeometry(t, target, 100, elapsed, dir)
}

func TestNoSolution(t *testing.T) {
	// 同速逃逸且继续向外加速：[0, MaxFlightMs] 内没有任何可行时刻。
	target := Target{
		P: Vec{X: 100},
		V: Vec{X: 100},
		A: Vec{X: 1000},
	}
	engine := &Engine{}
	elapsed, dir, ok := engine.Lead(target, 100)
	if ok {
		t.Fatalf("无解目标必须返回 ok=false，却给了 t=%v", elapsed)
	}
	if elapsed != 0 || dir != (Vec{}) {
		t.Fatalf("无解时不应硬给时刻或方向：t=%v dir=%+v", elapsed, dir)
	}
	if engine.Steps() > MaxIter {
		t.Fatalf("单次解算求值 %d 次，超过 MaxIter=%d", engine.Steps(), MaxIter)
	}
}

func TestAlreadyInsideRadiusAtZero(t *testing.T) {
	// 零时刻目标已经在半径内：直接返回 0，方向取当前位置方向。
	target := Target{P: Vec{X: 0.3, Y: -0.4}}
	engine := &Engine{}
	elapsed, dir, ok := engine.Lead(target, 100)
	if !ok || elapsed != 0 {
		t.Fatalf("零时刻在半径内应返回 t=0：t=%v ok=%v", elapsed, ok)
	}
	if !approxVec(dir, Vec{X: 0.6, Y: -0.8}, 1e-12) {
		t.Fatalf("零时刻方向应指向当前目标位置：%+v", dir)
	}
}

func TestDirectionPointsAtHitPosition(t *testing.T) {
	// 横向运动目标：开火方向必须指向命中时刻的位置，而不是初始位置。
	target := Target{P: Vec{X: 100}, V: Vec{Y: 150}}
	engine := &Engine{}
	elapsed, dir, ok := engine.Lead(target, 250)
	if !ok {
		t.Fatal("横向运动目标应该有解")
	}
	if dir.Y <= 0 || !approxEqual(math.Hypot(dir.X, dir.Y), 1, 1e-12) {
		t.Fatalf("开火方向应带有命中时刻的横向分量：%+v t=%v", dir, elapsed)
	}
	assertLeadGeometry(t, target, 250, elapsed, dir)
}

func TestStepBudgetAcrossManySolves(t *testing.T) {
	engine := &Engine{}
	for index := 0; index < 1000; index++ {
		engine.Lead(Target{P: Vec{X: 100}}, 100)
	}
	if engine.Steps() > 64000 {
		t.Fatalf("一千次解算求值 %d 次，应在 64000 以内", engine.Steps())
	}
}

// referenceLead 是独立参考实现：以 0.02ms 密扫可行域
// g(t)=|Q(t)|-弹速*t-HitRadius，对穿越段二分、对贴合段做三元搜索，
// 与被测实现的多项式求根路径完全不同。
func referenceLead(target Target, bulletSpeed float64) (float64, Vec, bool) {
	g := func(t float64) float64 {
		position := hitPosition(target, t)
		return math.Hypot(position.X, position.Y) - bulletSpeed*t - HitRadius
	}
	if g(0) <= 1e-12 {
		return 0, direction(hitPosition(target, 0)), true
	}
	tMax := MaxFlightMs / 1000
	const step = 2e-5
	tPrev, gPrev := 0.0, g(0)
	for t := step; ; t += step {
		if t > tMax {
			t = tMax
		}
		gNow := g(t)
		if gNow <= 0 {
			lo, hi := tPrev, t
			for iteration := 0; iteration < 80; iteration++ {
				mid := (lo + hi) * 0.5
				if g(mid) <= 0 {
					hi = mid
				} else {
					lo = mid
				}
			}
			return hi * 1000, direction(hitPosition(target, hi)), true
		}
		if gPrev > 0 && gNow > 0 && g((tPrev+t)*0.5) < 1e-2 {
			// 端点都不可行时，检查小区间内部是否有贴合（相切）可行点。
			a, b := tPrev, t
			for iteration := 0; iteration < 80; iteration++ {
				m1 := a + (b-a)/3
				m2 := b - (b-a)/3
				if g(m1) < g(m2) {
					b = m2
				} else {
					a = m1
				}
			}
			best := (a + b) * 0.5
			if g(best) <= 1e-9 {
				return best * 1000, direction(hitPosition(target, best)), true
			}
		}
		if t >= tMax {
			break
		}
		tPrev, gPrev = t, gNow
	}
	return 0, Vec{}, false
}

func TestMatchesReferenceOn200RandomCases(t *testing.T) {
	random := rand.New(rand.NewSource(20260930))
	hits, misses := 0, 0
	for index := 0; index < 200; index++ {
		angle := random.Float64() * 2 * math.Pi
		distance := 1 + random.Float64()*199
		target := Target{
			P: Vec{X: distance * math.Cos(angle), Y: distance * math.Sin(angle)},
			V: Vec{X: (random.Float64()*2 - 1) * 80, Y: (random.Float64()*2 - 1) * 80},
			A: Vec{X: (random.Float64()*2 - 1) * 120, Y: (random.Float64()*2 - 1) * 120},
		}
		bulletSpeed := 5 + random.Float64()*145

		engine := &Engine{}
		gotT, gotDir, gotOK := engine.Lead(target, bulletSpeed)
		wantT, wantDir, wantOK := referenceLead(target, bulletSpeed)

		if gotOK != wantOK {
			t.Fatalf("case %d 命中判定不一致：got=%v want=%v target=%+v speed=%v",
				index, gotOK, wantOK, target, bulletSpeed)
		}
		if engine.Steps() > MaxIter {
			t.Fatalf("case %d 单次解算求值 %d 次，超过 MaxIter=%d",
				index, engine.Steps(), MaxIter)
		}
		if gotOK {
			hits++
			if !approxEqual(gotT, wantT, 1e-3) {
				t.Fatalf("case %d 命中时刻差异超 1e-3ms：got=%v want=%v",
					index, gotT, wantT)
			}
			if !approxVec(gotDir, wantDir, 1e-6) {
				t.Fatalf("case %d 开火方向不一致：got=%+v want=%+v",
					index, gotDir, wantDir)
			}
		} else {
			misses++
		}
	}
	if hits == 0 || misses == 0 {
		t.Fatalf("随机样本应同时覆盖命中与无解：hits=%d misses=%d", hits, misses)
	}
}
