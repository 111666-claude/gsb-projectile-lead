package projectile

import (
	"math"
	"testing"
)

func TestStaticTargetTime(t *testing.T) {
	engine := &Engine{}
	elapsed, dir, ok := engine.Lead(Vec{X: 100}, Vec{}, 100)
	if !ok {
		t.Fatal("静止目标应该能解出命中时间")
	}
	if math.Abs(elapsed-1000) > 1e-6 {
		t.Fatalf("静止目标 100 米、弹速 100 米每秒应该是 1000ms：%v", elapsed)
	}
	if math.Abs(dir.X-1) > 1e-9 || math.Abs(dir.Y) > 1e-9 {
		t.Fatalf("方向应该朝 +X：%+v", dir)
	}
}

func TestTimeIsPositive(t *testing.T) {
	if elapsed, _, _ := (&Engine{}).Lead(Vec{X: 10}, Vec{}, 10); elapsed <= 0 {
		t.Fatalf("命中时间应该为正：%v", elapsed)
	}
}

func TestStepsStartAtZero(t *testing.T) {
	if (&Engine{}).Steps() != 0 {
		t.Fatal("初始步数应该是 0")
	}
}

func TestConstants(t *testing.T) {
	if StepMs != 10 || MaxSimMs != 10000 {
		t.Fatal("弹道常量被改了")
	}
}
