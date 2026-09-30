# projectile-lead

弹道预判：目标带加速度运动，要在给定弹速与命中半径下找出最早的可行开火时刻，只用 Go 标准库。

```
go test ./...
go run ./cmd/projectile-lead --sample radius
go run ./cmd/projectile-lead --sample accel
go run ./cmd/projectile-lead --sample noway
```

## 口径（README 为准）

- **目标运动**：位置是 `P + V*t + A*t²/2`（`t` 以秒计），速度与加速度恒定。
- **命中判定带半径**：子弹以固定速度沿直线飞行，命中条件是子弹在时刻 `t` 与目标
  **相距不超过 `HitRadius`**（不是打中一个点）。
- **最早可行时刻**：返回满足条件的最小 `t`（毫秒），并把开火方向取成命中时刻目标位置的方向；
  `t = 0` 时目标已经在半径内就返回 0。
- **精度**：返回时刻与真实最早可行时刻之差不超过 1e-3 毫秒。
- **无解要报**：`[0, MaxFlightMs]` 内没有任何可行时刻时返回 `ok=false`，不许硬给一个答案。
- **查询代价**：`Steps` 统计可行域求值次数，一次解算不超过 `MaxIter` 次，
  不许按飞行时间逐帧模拟。
- **规模**：每秒 100 帧、每帧 10 万次解算，单次 ≤ `MaxIter` 次求值，内存 O(1)。

## 输出契约（不改格式）

```
hit=true t=995.0 steps<=64
hit=true t=615.8
hit=false t=0.0
steps<=64000
```

## 常量

```
SimStepMs   = 10
MaxFlightMs = 5000
HitRadius   = 0.5
MaxIter     = 64
```

## 目录

```
lead.go                 弹道解算
cmd/projectile-lead     命令行入口
lead_test.go            go test 用例
```
