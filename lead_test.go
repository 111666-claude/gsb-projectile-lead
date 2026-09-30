package projectile

import (
	"math"
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
